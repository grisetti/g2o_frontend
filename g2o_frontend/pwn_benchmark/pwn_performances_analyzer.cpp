#include "pwn_performances_analyzer.h"

namespace pwn {
  PWNPerformancesAnalyzer::PWNPerformancesAnalyzer() : PerformancesAnalyzer() {
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
    
    // Correspondece finder and linearizer
    _correspondenceFinder = new CorrespondenceFinder();
    _correspondenceFinder->setInlierDistanceThreshold(0.5f);
    _correspondenceFinder->setFlatCurvatureThreshold(0.02f);  
    _correspondenceFinder->setInlierCurvatureRatioThreshold(1.3f);
    _correspondenceFinder->setInlierNormalAngularThreshold(cosf(M_PI / 6.0f));
    _linearizer = new Linearizer();
    _linearizer->setInlierMaxChi2(1e3);
    
#ifdef _PWN_USE_CUDA_
    _aligner = new CuAligner();
#else
    _aligner = new Aligner();
#endif

    _aligner->setProjector(_projector);
    _aligner->setLinearizer(_linearizer);
    _linearizer->setAligner(_aligner);
    _aligner->setCorrespondenceFinder(_correspondenceFinder);
    _aligner->setInnerIterations(1);
    _aligner->setOuterIterations(10);
    _merger = new Merger();
    _merger->setDepthImageConverter(_converter);
    _merger->setMaxPointDepth(6.0f);
    _merger->setDistanceThreshold(0.5f);
    _merger->setNormalThreshold(cosf(M_PI / 6.0f));

    _referenceDepthFilename = ""; 
    _currentDepthFilename = "";
    _scaleFactor = 0.001f;
    _chunkStep = 30;
    _counter = 0;
    _localPose = Isometry3f::Identity();
    _chunkInitialPose = Isometry3f::Identity();
  }

  PWNPerformancesAnalyzer::~PWNPerformancesAnalyzer() {

  }

  bool PWNPerformancesAnalyzer::loadNextPair() {
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
      sprintf(name, "part-%04d.pwn", _counter);
      _scene.save(name, 5, true, _chunkInitialPose);
      _scene.clear();
      _localPose = Isometry3f::Identity();
      _chunkInitialPose = _absolutePose;
    }
    _scene.add(_referenceFrame, _localPose);
    _merger->merge(&_scene, _localPose * _sensorOffset);
    _counter++;

    return true;
  }
  
  void PWNPerformancesAnalyzer::computeAlignment() {
    cout << "-------------------------------------------" << endl;

    // Load depth and color images   
    _referenceDepth.load(_referenceDepthFilename.c_str());
    _currentDepth.load(_currentDepthFilename.c_str());
    _imageRows = _referenceDepth.rows();
    _imageCols = _referenceDepth.cols();
    _scaledImageRows = _imageRows / _scale;
    _scaledImageCols = _imageCols / _scale;
    _correspondenceFinder->setImageSize(_scaledImageRows, _scaledImageCols);
    _dummyCorrespondenceFinder->setImageSize(_scaledImageRows, _scaledImageCols);
    _projector->setImageSize(_scaledImageRows, _scaledImageCols);

    // Align clouds
    Isometry3f initialGuess = Isometry3f::Identity();
    initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    if(_indexImage.cols() != _imageCols || _indexImage.rows() != _imageRows) 
      _indexImage.resize(_imageRows, _imageCols);
    if(_scaledIndexImage.cols() != _scaledImageCols || _scaledIndexImage.rows() != _scaledImageRows) 
      _scaledIndexImage.resize(_scaledImageRows, _scaledImageCols);
    _projector->setTransform(_localPose * _sensorOffset);
    _projector->project(_scaledIndexImage, _referenceScaledDepth, _scene.points());    
    _converter->compute(_subScene, _referenceScaledDepth, _sensorOffset, false);
    _aligner->setReferenceFrame(&_subScene);
    _projector->setTransform(Isometry3f::Identity());

    //_aligner->setReferenceFrame(&_referenceFrame);
    _aligner->setCurrentFrame(&_currentFrame);
    _aligner->setInitialGuess(initialGuess);
    _aligner->setSensorOffset(_sensorOffset);
    _ostart = get_time();
    _aligner->align();
    _oend = get_time();

    // Update transformations
    _relativePose = _aligner->T();
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
    // currFrame.save("alignedPWN.pwn", 1, true, currentRelativePose);
  }
}
