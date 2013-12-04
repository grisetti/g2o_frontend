#pragma once

#include "g2o_frontend/pwn_core/frame.h"

#include "g2o_frontend/basemath/bm_se3.h"

namespace pwn {

  class OdometryController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    OdometryController() {}
    virtual ~OdometryController() { }
    
    // Input and output file initialization
    virtual bool fileInitialization(const char *groundtruthFilename_, const char *associationsFilename_,
				    const char *trajectoryFilename_, const char *benchmarkFilename_);
    // Ground truth starting pose extrapolation
    virtual void getGroundTruthPose(Eigen::Isometry3f &pose, const double poseTimestamp);
    // Load next frame
    virtual bool loadFrame(Frame *&frame); 
    // Procces current frame
    virtual bool processFrame() { 
      std::cout << "WARNING: you should overwrite this method" << std::endl; 
      return false;
    }
    // Write current result
    virtual void writeResults() { std::cout << "WARNING: you should overwrite this method" << std::endl; }

  protected:
    // Filenames
    string _groundtruthFilename, _associationsFilename, _trajectoryFilename, _benchmarkFilename;
    // File streams
    ifstream _ifsGroundtruth, _ifsAssociations;
    ofstream _ofsTrajectory, _ofsBenchmark;
  };

}
