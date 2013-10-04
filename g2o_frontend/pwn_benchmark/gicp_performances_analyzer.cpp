#include "gicp_performances_analyzer.h"
#include "g2o_frontend/pwn_utils/pwn_file_format_converter.h"

#include <pcl/registration/gicp.h>

namespace pwn {
  GICPPerformancesAnalyzer::GICPPerformancesAnalyzer() : PerformancesAnalyzer() {
    // Stats calculator
    _statsCalculator = new StatsCalculator();
    _statsCalculator->setMinImageRadius(10);
    _statsCalculator->setMinImageRadius(30);
    _statsCalculator->setMinPoints(50);
    _statsCalculator->setWorldRadius(0.05f);
    _statsCalculator->setCurvatureThreshold(1.0f);

    // Information matrix calculators
    _pointInformationMatrixCalculator = new PointInformationMatrixCalculator();
    _normalInformationMatrixCalculator = new NormalInformationMatrixCalculator();
    _pointInformationMatrixCalculator->setCurvatureThreshold(1.0f);
    _normalInformationMatrixCalculator->setCurvatureThreshold(1.0f);

    // Depth image converter
    _converter = new DepthImageConverter(_projector, _statsCalculator, 
					_pointInformationMatrixCalculator, _normalInformationMatrixCalculator);
    
    _referenceDepthFilename = ""; 
    _currentDepthFilename = "";
    _scaleFactor = 0.001f;
    _chunkStep = 30;
    _counter = 0;
    _localPose = Isometry3f::Identity();
    _chunkInitialPose = Isometry3f::Identity();
  }

  GICPPerformancesAnalyzer::~GICPPerformancesAnalyzer() {

  }

  bool GICPPerformancesAnalyzer::loadNextPair() {
    _referenceDepthFilename = _currentDepthFilename;

    // Read a line from associations
    char line[4096];    
    if (!_isAssociations.getline(line, 4096)) {
      cout << "No more depth images to analyze." << endl;
      return false;
    }
    string dummy;
    istringstream issAssociations(line);
    if (_referenceDepthFilename == "") {
      string initialTimestamp;
      issAssociations >> initialTimestamp >> dummy;
      issAssociations >> dummy >> _referenceDepthFilename;
      if (!_isAssociations.getline(line, 4096)) {
	cout << "Only one depth image can be read." << endl;
	return false;
      }
      parseGroundTruth(_absolutePose, _isGroundtruth, atof(initialTimestamp.c_str()));    
      _chunkInitialPose = _absolutePose;
      istringstream issAssociations(line);
      issAssociations >> _currentTimestamp >> dummy;
      issAssociations >> dummy >> _currentDepthFilename;
    }
    else {
      issAssociations >> _currentTimestamp >> dummy;
      issAssociations >> dummy >> _currentDepthFilename;
    }

    cout << "Scale factor: " << _scaleFactor << endl;
    // Load depth images      
    _referenceDepth.load((_referenceDepthFilename.substr(0, _referenceDepthFilename.size() - 3) + "pgm").c_str(), true, _scaleFactor);
    _currentDepth.load((_currentDepthFilename.substr(0, _currentDepthFilename.size() - 3) + "pgm").c_str(), true, _scaleFactor);

    // Compute stats
    DepthImage::scale(_referenceScaledDepth, _referenceDepth, _scale);
    DepthImage::scale(_currentScaledDepth, _currentDepth, _scale);
    _converter->compute(_referenceFrame, _referenceScaledDepth, _sensorOffset, false);
    _converter->compute(_currentFrame, _currentScaledDepth, _sensorOffset, false);

    if(_counter % _chunkStep == 0 && _counter != 0) {
      char name[1024];
      sprintf(name, "gicp-part-%04d.pwn", _counter);
      _scene.save(name, 5, true, _chunkInitialPose);
      _scene.clear();
      _localPose = Isometry3f::Identity();
      _chunkInitialPose = _absolutePose;
    }
    _scene.add(_referenceFrame, _localPose);
    _counter++;

    return true;
  }
  
  void GICPPerformancesAnalyzer::computeAlignment() {
    cout << "-------------------------------------------" << endl;

    // Load depth and color images   
    _imageRows = _referenceDepth.rows();
    _imageCols = _referenceDepth.cols();
    _scaledImageRows = _imageRows / _scale;
    _scaledImageCols = _imageCols / _scale;
    _dummyCorrespondenceFinder->setImageSize(_scaledImageRows, _scaledImageCols);
    _projector->setImageSize(_scaledImageRows, _scaledImageCols);

    // Align clouds
    Isometry3f initialGuess = Isometry3f::Identity();
    initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    // Convert pwn clouds to pcl type
    pcl::PointCloud<pcl::PointXYZ>::Ptr refPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    pwnToPclPointCloud(refPclPointCloud, &_referenceFrame);
    pwnToPclPointCloud(currPclPointCloud, &_currentFrame);   
    
    cout << "Aligning: " << _currentDepthFilename << " --> " << _referenceDepthFilename << endl;
    cout << "Ref Size: " << refPclPointCloud->points.size() << " --- " << _referenceFrame.points().size() << endl;
    cout << "CUr Size: " << currPclPointCloud->points.size() << " --- " << _currentFrame.points().size() << endl;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    // Set the maximum correspondence distance
    gicp.setMaxCorrespondenceDistance(0.5);
    // Set the maximum number of iterations
    // gicp.setMaximumIterations(50);
    // Set the transformation epsilon
    // gicp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon
    // gicp.setEuclideanFitnessEpsilon(1);
    gicp.setInputSource(currPclPointCloud);
    gicp.setInputTarget(refPclPointCloud);
    _ostart = get_time();
    gicp.align(aligned);
    _oend = get_time();

    Matrix4f finalTransformation = gicp.getFinalTransformation();
    Isometry3f T = Isometry3f::Identity();
    T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    T.translation() = finalTransformation.block<3, 1>(0, 3);
    T.linear() = finalTransformation.block<3, 3>(0, 0);

    // Update transformations
    _relativePose = T;
    _relativePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _absolutePose = _absolutePose * _relativePose;
    _absolutePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _localPose = _localPose * _relativePose;
    _localPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
	
    // Initialize dummy aligner
    _dummyAligner->setReferenceFrame(&_referenceFrame);
    _dummyAligner->setCurrentFrame(&_currentFrame);
    _dummyAligner->setInitialGuess(_relativePose);
    _dummyAligner->setSensorOffset(_sensorOffset);

    //_currentFrame.save((_currentDepthFilename.substr(0, _currentDepthFilename.size() - 3) + "pwn").c_str(), 10, true, _absolutePose);   

    // Save clouds
    // refFrame.save("reference.pwn", 1, true, Eigen::Isometry3f::Identity());
    // currFrame.save("current.pwn", 1, true, Eigen::Isometry3f::Identity());      
    // currFrame.save("alignedGICP.pwn", 1, true, currentRelativePose);
  }
}
