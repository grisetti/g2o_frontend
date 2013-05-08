#include "homogeneousvector4f.h"
#include "homogeneouspoint3fomega.h"
#include "g2o_frontend/basemath/bm_se3.h"

using namespace Eigen;

class Aligner;

class Linearizer {
 public:
  Linearizer() {
    _aligner = 0;
    _H.setZero();
    _b.setZero();
    _inlierMaxChi2 = 9e3;
  }
  
  inline void setAligner(Aligner * const aligner_) { _aligner = aligner_; }
  inline void setT(const Isometry3f T_) { 
    _T = T_; 
    _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1; 
  }
  inline void setInlierMaxChi2(const float inlierMaxChi2_) { _inlierMaxChi2 = inlierMaxChi2_; }

  inline Aligner *aligner() const { return _aligner; }  
  inline Isometry3f T() const { return _T; }  
  inline float inlierMaxChi2() const { return _inlierMaxChi2; }
  inline Matrix6f H() const { return _H; }  
  inline Vector6f b() const { return _b; }  

  float update();
 
 protected:
  Aligner *_aligner;

  Isometry3f _T;
  float _inlierMaxChi2;

  Matrix6f _H;
  Vector6f _b;
};


