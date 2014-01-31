#pragma once

#include <fstream>

#include "g2o_frontend/pwn_boss/cloud.h"

#include "g2o_frontend/basemath/bm_se3.h"

namespace pwn {

  class OdometryController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    OdometryController() {}
    virtual ~OdometryController() {}
    
    // Input and output file initialization
    virtual bool fileInitialization(const char *groundtruthFilename_, const char *associationsFilename_,
				    const char *trajectoryFilename_, const char *benchmarkFilename_);
    // Ground truth starting pose extrapolation
    virtual void getGroundTruthPose(Eigen::Isometry3f &pose, const double poseTimestamp);
    // Load next frame
    virtual bool loadCloud(Cloud *&/*cloud*/) {
      std::cout << "WARNING: you should overwrite this method" << std::endl; 
      return false;
    }
    // Procces current frame
    virtual bool processCloud() { 
      std::cout << "WARNING: you should overwrite this method" << std::endl; 
      return false;
    }
    // Write current result
    virtual void writeResults() { std::cout << "WARNING: you should overwrite this method" << std::endl; }

  protected:
    // Filenames
    std::string _groundtruthFilename, _associationsFilename, _trajectoryFilename, _benchmarkFilename;
    // File streams
    std::ifstream _ifsGroundtruth, _ifsAssociations;
    std::ofstream _ofsTrajectory, _ofsBenchmark;
  };

}
