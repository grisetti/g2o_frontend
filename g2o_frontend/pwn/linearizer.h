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
  
  inline const Eigen::MatrixXf H() const { return _H; }
  inline const Eigen::VectorXf b() const { return _b; }
  inline void setAligner(Aligner* aligner_) { _aligner = aligner_; }

  inline float inlierMaxChi2() const { return _inlierMaxChi2; }
  inline void setInlierMaxChi2(float inlierMaxChi2_) { _inlierMaxChi2 = inlierMaxChi2_; }

  inline void _computeHb_qt(Matrix6f& H, Vector6f b, 
			    const HomogeneousPoint3f& point, const HomogeneousNormal3f& normal, 
			    const HomogeneousPoint3f& pointError, const HomogeneousNormal3f& normalError, 
			    HomogeneousPoint3fOmega& pointOmega, HomogeneousPoint3fOmega& normalOmega) {
    const Vector3f& p = point.head<3>();
    const Vector3f& n = normal.head<3>();
    Matrix3f Sp = skew(p);
    Matrix3f Sn = skew(n);
    H.block<3,3>(0,0) += Sp.transpose()*pointOmega.block<3,3>(0,0)*Sp + Sn.transpose()*normalOmega.block<3,3>(0,0)*Sn;
    H.block<3,3>(0,3) += -Sp.transpose()*pointOmega.block<3,3>(0,0);
    H.block<3,3>(3,3) += pointOmega.block<3,3>(0,0);
    const Vector3f ep = pointOmega.block<3,3>(0,0)*pointError.head<3>();
    const Vector3f en = normalOmega.block<3,3>(0,0)*normalError.head<3>();
    b.head<3>() += -Sp.transpose()*ep + Sn.transpose()*en;
    b.tail<3>() += ep;
  }

  void update();
 
 protected:
  Aligner* _aligner;
  Matrix6f _H;
  Vector6f _b;

  float _inlierMaxChi2;
};


