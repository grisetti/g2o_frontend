#include "gicp_odometry_controller.h"

#include "g2o_frontend/pwn_utils/pwn_file_format_converter.h"

#include <pcl/registration/gicp.h>

namespace pwn {

  GICPOdometryController::GICPOdometryController(const char *configFilename, const char *logFilename) {
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
    _relativePose = Eigen::Isometry3f::Identity();
    _relativePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _referencePose = Eigen::Isometry3f::Identity();
    _referencePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _localPose = Eigen::Isometry3f::Identity();
    _localPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    _referenceFrame = 0;
    _currentFrame = 0;
    _converter = 0;
    
    // Read pwn configuration file
    cout << "Loading pwn configuration file..." << endl;
    std::vector<boss::Serializable*> instances = readGICPConfigFile(configFilename);
    cout << "... done" << endl;

    _scene = new Frame();
    _subScene = new Frame();
    _merger = new Merger();
    _merger->setDepthImageConverter(_converter);
    _merger->setMaxPointDepth(6.0f);
    _merger->setDistanceThreshold(0.5f);
    _merger->setNormalThreshold(cosf(M_PI / 6.0f));

    // Create pwn log file
    cout << "Creating GICP log file \'" << logFilename <<"\'... ";
    _ofsLog.open(logFilename);
    if (!_ofsLog) {
      cerr << "Impossible to create the GICP log file containing the trajectory computed" << endl;
    }
    cout << "done." << endl;

    update();
  }

  GICPOdometryController::~GICPOdometryController() {}

  bool GICPOdometryController::loadFrame(Frame *&frame) {
    // Read a line from associations
    char line[4096];
    if (!_ifsAssociations.getline(line, 4096)) {
      char name[1024];
      sprintf(name, "./depth/gicp-part-%05d.pwn", _counter);
      _scene->save(name, 1, true, _referencePose);
      _scene->clear();
      _localPose = Isometry3f::Identity();
      _referencePose = _globalPose;
      return false;
    }
    istringstream issAssociations(line);
    string dummy;
    issAssociations >> _timestamp >> dummy;
    issAssociations >> dummy >> _depthFilename;

    cout << "loading: " << _depthFilename << endl;
    // Load depth image      
    if (!_depthImage.load((_depthFilename.substr(0, _depthFilename.size() - 3) + "pgm").c_str(), true, _scaleFactor)) {
      return false;
    }
    DepthImage::scale(_scaledDepthImage, _depthImage, _scale);
    
    // Compute cloud
    update();
    PinholePointProjector *projector = dynamic_cast<PinholePointProjector*>(_converter->projector());
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
      sprintf(name, "./depth/gicp-part-%05d.pwn", _counter);
      _scene->save(name, 1, true, _referencePose);
      _scene->clear();
      _localPose = Isometry3f::Identity();
      _referencePose = _globalPose;
      _scene->add(*_referenceFrame, _localPose);
    }
    _counter++;

    return true;
  }

  bool GICPOdometryController::processFrame() {
    if(!_referenceFrame || !_currentFrame) {
      return false;
    }
    
    std::cout << "****************** Aligning frame " << _depthFilename << " ******************" << std::endl; 

    if(_scaledIndexImage.cols() != _scaledImageCols || _scaledIndexImage.rows() != _scaledImageRows) 
      _scaledIndexImage.resize(_scaledImageRows, _scaledImageCols);
    _converter->projector()->setTransform(_localPose * _sensorOffset);
    _converter->projector()->project(_scaledIndexImage, _scaledDepthImage, _scene->points());    
    _converter->compute(*_subScene, _scaledDepthImage, _sensorOffset);
    _converter->projector()->setTransform(Isometry3f::Identity());

    // Convert pwn clouds to pcl type
    pcl::PointCloud<pcl::PointXYZ>::Ptr refPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    // _scene->transformInPlace(_relativePose.inverse());
    // pwnToPclPointCloud(refPclPointCloud, _referenceFrame);
    // pwnToPclPointCloud(refPclPointCloud, _scene);
    pwnToPclPointCloud(refPclPointCloud, _subScene);
    pwnToPclPointCloud(currPclPointCloud, _currentFrame);   
    
    // Align clouds
    Isometry3f initialGuess = Isometry3f::Identity();
    initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    // Set the maximum correspondence distance
    // gicp.setMaxCorrespondenceDistance(0.5);
    // Set the maximum number of iterations
    // gicp.setMaximumIterations(50);
    // Set the transformation epsilon
    // gicp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon
    // gicp.setEuclideanFitnessEpsilon(1);
    
    gicp.setInputSource(currPclPointCloud);
    gicp.setInputTarget(refPclPointCloud);
    _ostart = g2o::get_time();
    gicp.align(aligned);
    _oend = g2o::get_time();

    // Update transformations
    Matrix4f finalTransformation = gicp.getFinalTransformation();
    _relativePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _relativePose.translation() = finalTransformation.block<3, 1>(0, 3);
    _relativePose.linear() = finalTransformation.block<3, 3>(0, 0);
    _globalPose = _globalPose * _relativePose;
    _globalPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _localPose = _localPose * _relativePose;
    _localPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    if(!_counter%50 && _counter != 0) {
      Eigen::Matrix3f R = _globalPose.linear();
      Eigen::Matrix3f E = R.transpose() * R;
      E.diagonal().array() -= 1;
      _globalPose.linear() -= 0.5 * R * E;
    }
    _globalPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    // Merge clouds
    _scene->add(*_currentFrame, _localPose);
    _merger->merge(_scene, _localPose * _sensorOffset);      

    // Save current frame for debug
    // _currentFrame->save(("gicp_" + _depthFilename.substr(0, _depthFilename.size() - 3) + "pwn").c_str(), 5, true, _globalPose);   
    
    return true;
  }

  std::vector<boss::Serializable*> GICPOdometryController::readGICPConfigFile(const char *configFilename) {
    _converter = 0;
    boss::Deserializer des;
    des.setFilePath(configFilename);
    boss::Serializable *s;
    std::vector<boss::Serializable*> instances;
    while ((s = des.readObject())) {
      instances.push_back(s);
      DepthImageConverter *conv = dynamic_cast<DepthImageConverter*>(s);
      if (conv) {      
	_converter = conv;
      }
    }
    return instances;
  }

  void GICPOdometryController::writeResults() {
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
    std::cout << "Relative pose: " << t2v(_relativePose).transpose() << std::endl;
  
    // Write processing time
    _ofsBenchmark << _oend - _ostart << std::endl;
    std::cout << "Time: " << _oend - _ostart << " seconds" << std::endl;	  
  }

  void GICPOdometryController::update() {
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
