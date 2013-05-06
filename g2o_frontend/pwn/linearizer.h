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
  
  
  inline void setH(Matrix6f H_) { _H = H_; }
  inline void setb(Vector6f b_) { _b = b_; }
  inline void setAligner(Aligner* aligner_) { _aligner = aligner_; }
  inline void setT(Isometry3f T_) { 
    _T = T_; 
    _T.matrix().block<1,4>(3,0) << 0,0,0,1; 
  }
  inline void setInlierMaxChi2(float inlierMaxChi2_) { _inlierMaxChi2 = inlierMaxChi2_; }

  inline Matrix6f& H() { return _H; }  
  inline Vector6f& b() { return _b; }  
  inline Aligner* aligner() { return _aligner; }
  inline Isometry3f T() { return _T; }  
  inline float inlierMaxChi2() const { return _inlierMaxChi2; }


  float update();
 
 protected:
  Aligner* _aligner;
  Isometry3f _T;
  float _inlierMaxChi2;

  Matrix6f _H;
  Vector6f _b;
};


