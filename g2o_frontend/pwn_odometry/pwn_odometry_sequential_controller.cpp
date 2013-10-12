#include "pwn_odometry_sequential_controller.h"

namespace pwn {

  PWNOdometrySequentialController::PWNOdometrySequentialController(const char *configFilename, const char *logFilename) {
    // Pwn objects init
    _newChunk = false;
    _chunkStep = 30;
    _counter = 0;
    _scaledImageRows = 0;
    _scaledImageCols = 0;
    _scale = 1.0f;
    _scaleFactor = 0.001f;
    _sensorType = "kinect";
    _sensorOffset = Eigen::Isometry3f::Identity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _startingPose = Eigen::Isometry3f::Identity();
    _startingPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _globalPose = Eigen::Isometry3f::Identity();
    _globalPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _referencePose = Eigen::Isometry3f::Identity();
    _referencePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _localPose = Eigen::Isometry3f::Identity();
    _localPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    _referenceFrame = 0;
    _currentFrame = 0;
    _converter = 0;
    _aligner = 0;  
  
    // Read pwn configuration file
    cout << "Loading pwn configuration file..." << endl;
    std::vector<boss::Serializable*> instances = readPWNConfigFile(configFilename);
    cout << "... done" << endl;

    _scene = new Frame();
    _merger = new Merger();
    _merger->setDepthImageConverter(_converter);
    _merger->setMaxPointDepth(6.0f);
    _merger->setDistanceThreshold(0.5f);
    _merger->setNormalThreshold(cosf(M_PI / 6.0f));

    // Create pwn log file
    cout << "Creating PWN log file \'" << logFilename <<"\'... ";
    _ofsLog.open(logFilename);
    if (!_ofsLog) {
      cerr << "Impossible to create the PWN log file containing the trajectory computed" << endl;
    }
    cout << "done." << endl;

    update();
  }

  PWNOdometrySequentialController::~PWNOdometrySequentialController() {}

  bool PWNOdometrySequentialController::loadFrame(Frame *&frame) {
    // Read a line from associations
    char line[4096];    
    if (!_ifsAssociations.getline(line, 4096)) {
      char name[1024];
      sprintf(name, "./depth/pwn-part-%05d.pwn", _counter);
      _scene->save(name, 1, true, _globalPose);
      _scene->clear();
      _localPose = Isometry3f::Identity();
      _referencePose = _globalPose;
      return false;
    }
    istringstream issAssociations(line);
    string dummy;
    issAssociations >> _timestamp >> dummy;
    issAssociations >> dummy >> _depthFilename;
    
    // Load depth image      
    if (!_depthImage.load((_depthFilename.substr(0, _depthFilename.size() - 3) + "pgm").c_str(), true, _scaleFactor)) {
      return false;
    }
    DepthImage::scale(_scaledDepthImage, _depthImage, _scale);
    
    // Compute cloud
    update();
    PinholePointProjector *projector = dynamic_cast<PinholePointProjector*>(_converter->_projector);
    projector->setCameraMatrix(_scaledCameraMatrix);
    projector->setImageSize(_scaledImageRows, _scaledImageCols);

    frame = new pwn::Frame();
    _converter->compute(*frame, _scaledDepthImage, _sensorOffset);

    if (!_currentFrame) {
      cout << "Starting timestamp: " << _timestamp << endl;
      getGroundTruthPose(_startingPose, atof(_timestamp.c_str()));
      _globalPose = _startingPose;
      _referencePose = _startingPose;
      _localPose = Eigen::Isometry3f::Identity();
      std::cout << "Starting pose: " << t2v(_startingPose).transpose() << std::endl;
      _scene->add(*frame, _localPose);
    }

    _referenceFrame = _currentFrame;
    _currentFrame = frame;

    if (_counter%_chunkStep == 0 && _counter != 0) {
      std::cout << "New chunk created" << std::endl;
      char name[1024];
      sprintf(name, "./depth/pwn-part-%05d.pwn", _counter);
      _scene->save(name, 1, true, _globalPose);
      _scene->clear();
      _localPose = Isometry3f::Identity();
      _referencePose = _globalPose;
      _scene->add(*_referenceFrame, _localPose);
    }
    _counter++;
    
    return true;
  }

  bool PWNOdometrySequentialController::processFrame() {
    if(!_referenceFrame || !_currentFrame) {
      return false;
    }
    
    std::cout << "****************** Aligning frame " << _depthFilename << " ******************" << std::endl; 

    CorrespondenceFinder *correspondenceFinder = dynamic_cast<CorrespondenceFinder*>(_aligner->correspondenceFinder());
    correspondenceFinder->setImageSize(_scaledImageRows, _scaledImageCols);

    // Align clouds
    Isometry3f initialGuess = Isometry3f::Identity();
    initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _scene->transformInPlace(_aligner->T().inverse());
    // _aligner->setReferenceFrame(_referenceFrame);
    _aligner->setReferenceFrame(_scene);
    _aligner->setCurrentFrame(_currentFrame);
    _aligner->setInitialGuess(initialGuess);
    _aligner->setSensorOffset(_sensorOffset);
    _ostart = g2o::get_time();
    _aligner->align();
    _oend = g2o::get_time();

    // Update transformations
    _globalPose = _globalPose * _aligner->T();
    _globalPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _localPose = _localPose * _aligner->T();
    _localPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    if(!_counter%50 && _counter != 0) {
      Eigen::Matrix3f R = _globalPose.linear();
      Eigen::Matrix3f E = R.transpose() * R;
      E.diagonal().array() -= 1;
      _globalPose.linear() -= 0.5 * R * E;
    }
    _globalPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    // Merge clouds
    _scene->add(*_currentFrame, _aligner->T());
    _merger->merge(_scene, _aligner->T() * _sensorOffset);      

    // Save current frame for debug
    // _currentFrame->save(("pwn_" + _depthFilename.substr(0, _depthFilename.size() - 3) + "pwn").c_str(), 10, true, _globalPose);   
    
    return true;
  }

  std::vector<boss::Serializable*> PWNOdometrySequentialController::readPWNConfigFile(const char *configFilename) {
    _converter = 0;
    _aligner = 0;
    boss::Deserializer des;
    des.setFilePath(configFilename);
    boss::Serializable *s;
    std::vector<boss::Serializable*> instances;
    while ((s = des.readObject())) {
      instances.push_back(s);
      Aligner *al = dynamic_cast<Aligner*>(s);
      if (al) {
	_aligner = al;
      }
      DepthImageConverter *conv = dynamic_cast<DepthImageConverter*>(s);
      if (conv) {      
	_converter = conv;
      }
    }
    return instances;
  }

  void PWNOdometrySequentialController::writeResults() {
    // Write out global pose
    Vector6f absolutePoseVector = t2v(_globalPose);
    float qw = sqrtf(1.0f - (absolutePoseVector[3]*absolutePoseVector[3] + 
			     absolutePoseVector[4]*absolutePoseVector[4] + 
			     absolutePoseVector[5]*absolutePoseVector[5]));
    _ofsTrajectory << _timestamp << " "
		   << absolutePoseVector[0] << " " << absolutePoseVector[1] << " " << absolutePoseVector[2] << " " 
		   << absolutePoseVector[3] << " " << absolutePoseVector[4] << " " << absolutePoseVector[5] << " " << qw
		   << endl;
    std::cout << "Global pose: " << absolutePoseVector.transpose() << std::endl;
    std::cout << "Relative pose: " << t2v(_aligner->T()).transpose() << std::endl;
  
    // Write processing time
    _ofsBenchmark << _oend - _ostart << std::endl;
    std::cout << "Time: " << _oend - _ostart << " seconds" << std::endl;	  
  }

  void PWNOdometrySequentialController::update() {
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
      cerr << "Unknown sensor type: [" << _sensorType << "]" << endl;
      cerr << "Kinect camera matrix will be used" << endl;
      _cameraMatrix << 
	525.0f,   0.0f, 319.5f,
        0.0f, 525.0f, 239.5f,
        0.0f,   0.0f,   1.0f;  
    }
    
    float invScale = 1.0f / _scale;
    _scaledCameraMatrix = _cameraMatrix * invScale;
    _scaledCameraMatrix(2, 2) = 1.0f;

    _scaledImageRows = _depthImage.rows() / _scale;
    _scaledImageCols = _depthImage.cols() / _scale;
    
  }
}
