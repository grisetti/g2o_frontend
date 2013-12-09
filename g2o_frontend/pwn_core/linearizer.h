#pragma once

#include "homogeneousvector4f.h"
#include "informationmatrix.h"
#include "g2o_frontend/basemath/bm_se3.h"

using namespace Eigen;

namespace pwn {

  class Aligner;

  class Linearizer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Linearizer();
    virtual ~Linearizer() {}

    inline Aligner *aligner() const { return _aligner; }  
    inline void setAligner(Aligner * const aligner_) { _aligner = aligner_; }

    inline Isometry3f T() const { return _T; }  
    inline void setT(const Isometry3f T_) { 
      _T = T_; 
      _T.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f; 
    }

    inline float inlierMaxChi2() const { return _inlierMaxChi2; }
    inline void setInlierMaxChi2(const float inlierMaxChi2_) { _inlierMaxChi2 = inlierMaxChi2_; }

    inline bool robustKernel() const { return _robustKernel; }
    inline void setRobustKernel(bool robustKernel_) { _robustKernel=robustKernel_; }
    
    inline Matrix6f H() const { return _H; }  
    inline Vector6f b() const { return _b; }  
    inline float error() const { return _error; }
    inline int inliers() const { return _inliers; }
    
    void update();

  protected:
    Aligner *_aligner;

    Isometry3f _T;
    float _inlierMaxChi2;

    Matrix6f _H;
    Vector6f _b;
    float _error;
    int _inliers;
    bool _robustKernel;
  };

}
