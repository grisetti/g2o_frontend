#include "performances_analyzer.h"

namespace pwn {
  PerformancesAnalyzer::PerformancesAnalyzer() {
    // cameraMatrix
    _sensorType = "kinect";
    _cameraMatrix.setIdentity();
    _cameraMatrix << 
      525.0f,   0.0f, 319.5f,
        0.0f, 525.0f, 239.5f,
        0.0f,   0.0f,   1.0f;  
    _scale = 1.0f;
    float invScale = 1.0f / _scale;
    _scaledCameraMatrix = _cameraMatrix * invScale;
    _scaledCameraMatrix(2, 2) = 1.0f;
    
    // Projector
    _projector = new PinholePointProjector();
    _projector->setCameraMatrix(_scaledCameraMatrix);
    _projector->setMaxDistance(6.0f);
    
    // Sensor offset
    _sensorOffset = Isometry3f::Identity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    // Depth image rows and cols
    _imageRows = 640;
    _imageCols = 480;
    _scaledImageRows = _imageRows / _scale;
    _scaledImageCols = _imageCols / _scale;

    // Dummy correspondences finder, linearizer and aligner
    _dummyCorrespondenceFinder = new CorrespondenceFinder();
    _dummyCorrespondenceFinder->setInlierDistanceThreshold(0.5f);
    _dummyCorrespondenceFinder->setFlatCurvatureThreshold(1.0f);  
    _dummyCorrespondenceFinder->setInlierCurvatureRatioThreshold(std::numeric_limits<float>::max());
    _dummyCorrespondenceFinder->setInlierNormalAngularThreshold(cosf(M_PI));
    _dummyCorrespondenceFinder->setSize(_scaledImageRows, _scaledImageCols);
    
    _dummyLinearizer = new Linearizer();
    _dummyLinearizer->setInlierMaxChi2(1e3);        


#ifdef _PWN_USE_CUDA_
    _dummyAligner = new CuAligner();
#else
    _dummyAligner = new Aligner();
#endif
    _dummyAligner->setProjector(_projector);
    _dummyAligner->setLinearizer(_dummyLinearizer);
    _dummyLinearizer->setAligner(_dummyAligner);
    _dummyAligner->setCorrespondenceFinder(_dummyCorrespondenceFinder);
    _dummyAligner->setInnerIterations(0);
    _dummyAligner->setOuterIterations(1);

    _ostart = 0.0;
    _oend = 0.0;
  }

  PerformancesAnalyzer::~PerformancesAnalyzer() {

  }

  bool PerformancesAnalyzer::openEvaluationFiles(string groundtruthFilename, string associationsFilename,
						 string absoluteTrajectoryFilename, string relativeTrajectoryFilename,
						 string benchmarkStatsFilename) {
    // Create benchmark file
    cout << "Creating benchmark stats file \'" << benchmarkStatsFilename <<"\'... ";
    _osBenchmark.open(benchmarkStatsFilename.c_str());
    if (!_osBenchmark) {
      cerr << "Impossible to create the file containing the values of the benchmark." << endl;
      return false;
    }
    _osBenchmark << "inliers\tchi2\ttime" << endl;
    cout << "done." << endl;
  
    // Create absolute trajectory file
    cout << "Creating absolute trajectory file \'" << absoluteTrajectoryFilename <<"\'... ";
    _osAbsoluteTrajectory.open(absoluteTrajectoryFilename.c_str());
    if (!_osAbsoluteTrajectory) {
      cerr << "Impossible to create the file containing the absolute trajectory poses computed." << endl;
      return false;
    }
    cout << "done." << endl;
  
    // Create relative trajectory file
    cout << "Creating relative trajectory file \'" << relativeTrajectoryFilename <<"\'... ";
    _osRelativeTrajectory.open(relativeTrajectoryFilename.c_str());
    if (!_osRelativeTrajectory) {
      cerr << "Impossible to create the file containing the relative trajectory poses computed." << endl;
      return false;
    }
    cout << "done." << endl;

    // Open associations file
    cout << "Opening associations file \'" << associationsFilename <<"\'... ";
    _isAssociations.open("pwn_associations.txt");
    if (!_isAssociations) {
      cerr << "Impossible to open associations file." << endl;
      return false;
    }
    char line[4096];
    if (!_isAssociations.getline(line, 4096)) {
      cerr << "Associations file is empty." << endl;
      return false;
    }
    cout << "done." << endl;

    // Parsing ground truth file for the initial transformation
    cout << "Opening ground truth file \'" << groundtruthFilename <<"\'... ";
    _isGroundtruth.open("groundtruth.txt");
    if (!_isGroundtruth) {
      cerr << "Impossible to open ground truth file." << endl;
      return false;
    }
    cout << "done." << endl;

    return true;
  }

  void PerformancesAnalyzer::writePose() {
    // Write poses on files
    Vector6f absolutePoseVector = t2v(_absolutePose);    
    float qw = sqrtf(1.0f - (absolutePoseVector[3]*absolutePoseVector[3] + 
			     absolutePoseVector[4]*absolutePoseVector[4] + 
			     absolutePoseVector[5]*absolutePoseVector[5]));
    _osAbsoluteTrajectory << _currentTimestamp << " "
			  << absolutePoseVector[0] << " " << absolutePoseVector[1] << " " << absolutePoseVector[2] << " " 
			  << absolutePoseVector[3] << " " << absolutePoseVector[4] << " " << absolutePoseVector[5] << " " << qw
			  << endl;
    Vector6f relativePoseVector = t2v(_relativePose);
    qw = sqrtf(1.0f - (relativePoseVector[3]*relativePoseVector[3] + 
		       relativePoseVector[4]*relativePoseVector[4] + 
		       relativePoseVector[5]*relativePoseVector[5]));
    _osRelativeTrajectory << _currentTimestamp << " "
			  << relativePoseVector[0] << " " << relativePoseVector[1] << " " << relativePoseVector[2] << " " 
			  << relativePoseVector[3] << " " << relativePoseVector[4] << " " << relativePoseVector[5] << " " << qw
			  << endl;
    if(_verbose == 1) {
      cout << "Absolute Pose: " << absolutePoseVector.transpose() << endl;
      cout << "Relative Pose: " << relativePoseVector.transpose() << endl;
    }
  }

  void PerformancesAnalyzer::writeBenchmarkStats() {
    _dummyAligner->align();
    _dummyLinearizer->setT(_relativePose.inverse());
    _dummyLinearizer->computeChi2WithoutNormalsInfo();

    float chi2 = std::numeric_limits<float>::max();
    if (_dummyLinearizer->inliers() != 0)
      chi2 = _dummyLinearizer->computeChi2WithoutNormalsInfo() / _dummyLinearizer->inliers();
    _osBenchmark << _dummyLinearizer->inliers() << "\t";
    _osBenchmark << chi2 << "\t";
    _osBenchmark << _oend - _ostart << endl;

    if(_verbose == 1) {
      cout << "Time: " << _oend - _ostart << " seconds" << endl;	  
      cout << "Chi2: " << chi2 << endl;
      cout << "#Inliers: " << _dummyLinearizer->inliers() << endl;
    }
  }

  void PerformancesAnalyzer::parseGroundTruth(Isometry3f &initialTransformation, 
					      ifstream &isGroundtruth, double targetTimestamp) {
    char line[4096];
    double currentTimestamp;
    initialTransformation = Isometry3f::Identity();
    double minDifference = numeric_limits<float>::max();
    Vector6f initialTransformationVector;
    // Get target time stamp initial transformation
    while (isGroundtruth.getline(line, 4096)) {
      istringstream issGroundtruth(line);
      issGroundtruth >> currentTimestamp;
      if(fabs(currentTimestamp - targetTimestamp) < minDifference) {
	issGroundtruth >> initialTransformationVector[0] >> initialTransformationVector[1] >> initialTransformationVector[2]
		       >> initialTransformationVector[3] >> initialTransformationVector[4] >> initialTransformationVector[5];
	initialTransformation = v2t(initialTransformationVector);
	minDifference = fabs(currentTimestamp - targetTimestamp);
      }
    }
    initialTransformation.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  }

  void PerformancesAnalyzer::updateStructures() {
    _cameraMatrix.setIdentity();
    if (_sensorType == "xtion640") {
      _cameraMatrix << 
	570.342f,   0.0f,   320.0f,
        0,      570.342f, 240.0f,
        0.0f,     0.0f,     1.0f;  
    }  
    else if (_sensorType == "xtion320") {
      _cameraMatrix << 
	570.342f,  0.0f,   320.0f,
        0.0f,  570.342f, 240.0f,
        0.0f,    0.0f,     1.0f;  
      _cameraMatrix.block<2,3>(0,0) *= 0.5;
    } 
    else if (_sensorType == "kinect") {
      _cameraMatrix << 
	525.0f,   0.0f, 319.5f,
        0.0f, 525.0f, 239.5f,
        0.0f,   0.0f,   1.0f;  
    }
    else if (_sensorType == "kinectFreiburg1") {
      _cameraMatrix << 
	517.3f,   0.0f, 318.6f,
        0.0f, 516.5f, 255.3f,
        0.0f,   0.0f,   1.0f;  
    }
    else if (_sensorType == "kinectFreiburg2") {
      _cameraMatrix << 
	520.9f,   0.0f, 325.1f,
	0.0f,   521.0f, 249.7f,
	0.0f,     0.0f,   1.0f;  
    }
    else if (_sensorType == "kinectFreiburg3") {
      _cameraMatrix << 
	535.4f,   0.0f, 320.1f,
	0.0f,   539.2f, 247.6f,
	0.0f,     0.0f,   1.0f;  
    }
    else {
      cerr << "WARNING: ";
      cerr << "Unknown sensor type: [" << _sensorType << "]... aborting (you need to specify either xtion or kinect)." << endl;
      cerr << "Last camera matrix setted will be used" << endl;
    }

    float invScale = 1.0f / _scale;
    _scaledCameraMatrix = _cameraMatrix * invScale;
    _scaledCameraMatrix(2, 2) = 1.0f;
    _projector->setCameraMatrix(_scaledCameraMatrix);

    _scaledImageRows = _imageRows / _scale;
    _scaledImageCols = _imageCols / _scale;
    _dummyCorrespondenceFinder->setSize(_scaledImageRows, _scaledImageCols);
  }
}
