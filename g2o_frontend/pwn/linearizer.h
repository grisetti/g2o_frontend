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
  inline Vector6f& b() { return _b; }
  inline void setAligner(Aligner* aligner_) { _aligner = aligner_; }

  inline Isometry3f T() { return _T; }
  inline void setT(Isometry3f T_) { _T = T_; }

  inline float inlierMaxChi2() const { return _inlierMaxChi2; }
  inline void setInlierMaxChi2(float inlierMaxChi2_) { _inlierMaxChi2 = inlierMaxChi2_; }

  inline void _computeHb_qt(Matrix6f& H, Vector6f& b, 
			    const HomogeneousPoint3f& point, const HomogeneousNormal3f& normal, 
			    const HomogeneousPoint3f& pointError, const HomogeneousNormal3f& normalError, 
			    HomogeneousPoint3fOmega& pointOmega, HomogeneousPoint3fOmega& normalOmega) {
    const Vector3f& p = point.head<3>();
    const Vector3f& n = normal.head<3>();
    Matrix3f Sp = skew(p);
    Matrix3f Sn = skew(n);
    Matrix3f Omegap = pointOmega.block<3,3>(0,0);
    Matrix3f Omegan = normalOmega.block<3,3>(0,0); 
    H.block<3,3>(0,0) += Omegap;
    H.block<3,3>(0,3) += Omegap*Sp;
    H.block<3,3>(3,3) += Sp.transpose()*Omegap*Sp + Sn.transpose()*Omegan*Sn;
    const Vector3f ep = Omegap*pointError.head<3>();
    const Vector3f en = Omegan*normalError.head<3>();
    b.head<3>() += ep;
    b.tail<3>() += Sp.transpose()*ep + Sn.transpose()*en;
  }

  float update();
 
 protected:
  Aligner* _aligner;
  Isometry3f _T;

  Matrix6f _H;
  Vector6f _b;

  float _inlierMaxChi2;
};


