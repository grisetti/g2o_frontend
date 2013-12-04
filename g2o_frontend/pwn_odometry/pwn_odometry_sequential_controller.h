#pragma once

#include "g2o/stuff/timeutil.h"

#include "odometry_controller.h"

#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/pwn_core/aligner.h"
#include "g2o_frontend/pwn_core/merger.h"

namespace pwn {

  class PWNOdometrySequentialController : public OdometryController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PWNOdometrySequentialController(const char *configFilename_, const char *logFilename_);
    virtual ~PWNOdometrySequentialController();

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
    inline Eigen::Isometry3f relativePose() { return _aligner->T(); }
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
    Matrix3f _cameraMatrix, _scaledCameraMatrix;
    Eigen::Isometry3f _sensorOffset, _startingPose, _globalPose, _referencePose, _localPose;
    DepthImage _depthImage, _scaledDepthImage;
    MatrixXi _scaledIndexImage;
    Frame *_currentFrame, *_referenceFrame, *_scene, *_subScene;
    std::vector<boss::Serializable*> pwnStructures;
    DepthImageConverter *_converter;
    Aligner *_aligner;
    Merger *_merger;

    // Pwn configuration file reader
    std::vector<boss::Serializable*> readPWNConfigFile(const char *configFilename);
  };

}
