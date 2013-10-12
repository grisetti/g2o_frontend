#pragma once

#include <Eigen/Core>

#include "g2o/stuff/timeutil.h"

#include "odometry_controller.h"

#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/merger.h"

namespace pwn {

  class GICPOdometryController : public OdometryController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GICPOdometryController(const char *configFilename_, const char *logFilename_);
    virtual ~GICPOdometryController();

    // Load next frame
    virtual bool loadFrame(Frame *&frame);
    // Procces current frame
    virtual bool processFrame();
    // Write current result
    virtual void writeResults();

    // Update structures
    virtual void update();

    inline void setScaleFactor(const float scaleFactor_) { _scaleFactor = scaleFactor_; }
    inline void setScale(const float scale_) { 
      _scale = scale_;
      update();
    }
    inline void setSensorType(const string sensorType_) { 
      _sensorType = sensorType_;
      update();
    }
    inline void setChunkStep(int chunkStep_) { _chunkStep = chunkStep_; }
    
    inline Eigen::Isometry3f globalPose() { return _globalPose; }
    inline Eigen::Isometry3f relativePose() { return _relativePose; }
    inline int counter() { return _counter; }
    inline string timestamp() { return _timestamp; }
    inline Frame* referenceFrame() { return _referenceFrame; }
    inline Frame* currentFrame() { return _currentFrame; }

  protected:
    // File streams
    ofstream _ofsLog;

    // Pwn structures
    bool _newChunk;
    int _scaledImageRows, _scaledImageCols, _counter, _chunkStep; 
    float _scale, _scaleFactor;
    double _ostart, _oend;
    string _timestamp, _depthFilename, _sensorType;
    Eigen::Matrix3f _cameraMatrix, _scaledCameraMatrix;
    Eigen::Isometry3f _sensorOffset, _startingPose, _globalPose, _relativePose, _referencePose, _localPose;
    DepthImage _depthImage, _scaledDepthImage;
    Frame *_currentFrame, *_referenceFrame, *_scene;
    std::vector<boss::Serializable*> pwnStructures;
    DepthImageConverter *_converter;
    Merger *_merger;

    // Pwn configuration file reader
    std::vector<boss::Serializable*> readGICPConfigFile(const char *configFilename);
  };

}
