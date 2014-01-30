#pragma once

#include "g2o/stuff/timeutil.h"

#include "odometry_controller.h"

#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/pwn_boss/pinholepointprojector.h"
#include "g2o_frontend/pwn_boss/depthimageconverter.h"
#include "g2o_frontend/pwn_boss/aligner.h"
#include "g2o_frontend/pwn_boss/merger.h"

namespace pwn {

  class PWNOdometrySequentialController : public OdometryController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PWNOdometrySequentialController(const char *configFilename_, const char *logFilename_);
    virtual ~PWNOdometrySequentialController();

    // Load next cloud
    virtual bool loadCloud(Cloud *&cloud);
    // Procces current cloud
    virtual bool processCloud();
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
    inline Cloud* referenceCloud() { return _referenceCloud; }
    inline Cloud* currentCloud() { return _currentCloud; }

  protected:
    // Pwn configuration file reader
    std::vector<boss::Serializable*> readPWNConfigFile(const char *configFilename);

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
    RawDepthImage _rawDepthImage;
    DepthImage _depthImage, _scaledDepthImage;
    IntImage _scaledIndexImage;
    Cloud *_currentCloud, *_referenceCloud, *_scene, *_subScene;
    std::vector<boss::Serializable*> pwnStructures;
    pwn_boss::DepthImageConverter *_converter;
    pwn_boss::Aligner *_aligner;
    pwn_boss::Merger *_merger;
  };

}
