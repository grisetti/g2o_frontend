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
  
  inline Matrix6f& H() { return _H; }
  inline void setH(Matrix6f H_) { _H = H_; }
  inline Vector6f& b() { return _b; }
  inline void setb(Vector6f b_) { _b = b_; }
  inline Aligner* aligner() { return _aligner; }
  inline void setAligner(Aligner* aligner_) { _aligner = aligner_; }
  inline Isometry3f T() { return _T; }
  inline void setT(Isometry3f T_) { _T = T_; }
  inline float inlierMaxChi2() const { return _inlierMaxChi2; }
  inline void setInlierMaxChi2(float inlierMaxChi2_) { _inlierMaxChi2 = inlierMaxChi2_; }

  inline void _computeHb_tq(Matrix6f& H, Vector6f& b, 
			    const HomogeneousPoint3f& point, const HomogeneousNormal3f& normal, 
			    const HomogeneousPoint3f& pointError, const HomogeneousNormal3f& normalError, 
			    HomogeneousPoint3fOmega& pointOmega, HomogeneousPoint3fOmega& normalOmega) {
    const Vector3f& p = point.head<3>();
    const Vector3f& n = normal.head<3>();
    Matrix4f Sp = skew4(p);
    Matrix4f Sn = skew4(n);
    H.block<3,3>(0,0) += pointOmega.block<3,3>(0,0);
    H.block<3,3>(0,3) += (pointOmega*Sp).block<3,3>(0,0);
    H.block<3,3>(3,3) += (Sp.transpose()*pointOmega*Sp + Sn.transpose()*normalOmega*Sn).block<3,3>(0,0);
    const HomogeneousNormal3f ep = pointOmega*pointError;
    const HomogeneousNormal3f en = normalOmega*normalError;
    b.head<3>() += ep.head<3>();
    b.tail<3>() += (Sp.transpose()*ep + Sn.transpose()*en).head<3>();
  }

  float update();
 
 protected:
  Aligner* _aligner;
  Isometry3f _T;
  float _inlierMaxChi2;

  Matrix6f _H;
  Vector6f _b;
};


