#ifndef _PWN_LINEARIZER_H_
#define _PWN_LINEARIZER_H_
#include "homogeneousvector4f.h"
#include "informationmatrix.h"
#include "g2o_frontend/basemath/bm_se3.h"

using namespace Eigen;

namespace pwn {

class Aligner;

class Linearizer {
 public:
  Linearizer();  
  inline void setAligner(Aligner * const aligner_) { _aligner = aligner_; }
  inline void setT(const Isometry3f T_) { _T = T_; _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1; }
  inline void setInlierMaxChi2(const float inlierMaxChi2_) { _inlierMaxChi2 = inlierMaxChi2_; }

  inline Aligner *aligner() const { return _aligner; }  
  inline Isometry3f T() const { return _T; }  
  inline float inlierMaxChi2() const { return _inlierMaxChi2; }
  inline Matrix6f H() const { return _H; }  
  inline Vector6f b() const { return _b; }  
  inline float error() const { return _error;}
  inline int inliers() const { return _inliers;}
  void update();
 
 protected:
  Aligner *_aligner;

  Isometry3f _T;
  float _inlierMaxChi2;

  Matrix6f _H;
  Vector6f _b;
  float _error;
  int _inliers;
};

}

#endif
