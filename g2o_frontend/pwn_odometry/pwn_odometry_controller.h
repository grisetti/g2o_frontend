#pragma once

#include "g2o/stuff/timeutil.h"

#include "odometry_controller.h"

#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"

namespace pwn {

  class PWNOdometryController : public OdometryController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PWNOdometryController(const char *configFilename_, const char *logFilename_);
    virtual ~PWNOdometryController();

    // Load next frame
    virtual bool loadFrame(Frame *&frame);
    // Procces current frame
    virtual bool processFrame();
    // Write current result
    virtual void writeResults();

    // Update structures
    virtual void update();

    inline void setScaleFactor(const float scaleFactor_) { _scaleFactor = scaleFactor_; }
    inline void setInliersFraction(const float inliersFraction_) { _inliersFraction = inliersFraction_; }
    inline void setScale(const float scale_) { 
      _scale = scale_;
      update();
    }
    inline void setSensorType(const string sensorType_) { 
      _sensorType = sensorType_;
      update();
    }

    Eigen::Isometry3f globalPose() { return _globalPose; }
    Eigen::Isometry3f relativePose() { return _aligner->T(); }
    string timestamp() { return _timestamp; }
    Frame* referenceFrame() { return _referenceFrame; }
    Frame* currentFrame() { return _currentFrame; }

  protected:
    // File streams
    ofstream _ofsLog;

    // Pwn structures
    bool _updateReference;
    int _scaledImageRows, _scaledImageCols, _counter; 
    float _scale, _scaleFactor, _inliersFraction;
    double _ostart, _oend;
    string _timestamp, _depthFilename, _sensorType;
    Matrix3f _cameraMatrix, _scaledCameraMatrix;
    Eigen::Isometry3f _sensorOffset, _startingPose, _globalPose, _referencePose;
    DepthImage _depthImage, _scaledDepthImage;
    Frame *_currentFrame, *_referenceFrame;
    std::vector<boss::Serializable*> pwnStructures;
    Aligner *_aligner;
    DepthImageConverter *_converter;

    // Pwn configuration file reader
    std::vector<boss::Serializable*> readPWNConfigFile(const char *configFilename);
  };

}
