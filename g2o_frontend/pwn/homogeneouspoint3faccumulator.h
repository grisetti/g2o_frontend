#ifndef _HOMOGENEOUSPOINT3FACCUMULATOR_H_
#define _HOMOGENEOUSPOINT3FACCUMULATOR_H_

#include "homogeneousvector4f.h"

struct HomogeneousPoint3fAccumulator{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  inline HomogeneousPoint3fAccumulator(){
    clear();
  }
  inline void clear() {
    _sum.setZero();
    _squaredSum.setZero();
  }
  inline void operator +=(const HomogeneousPoint3fAccumulator& pa) {
    _sum+=pa._sum;
    _squaredSum+=pa._squaredSum;
  }
  inline void operator -=(const HomogeneousPoint3fAccumulator& pa) {
    _sum-=pa._sum;
    _squaredSum-=pa._squaredSum;
  }


  inline void operator +=(const HomogeneousPoint3f& v) {
    _sum+=(Eigen::Vector4f&)v;
    _squaredSum+=v*v.transpose();
  }

  inline HomogeneousPoint3f mean() const { 
    const float& d = _sum.coeff(3,0);
    if (d)
      return HomogeneousPoint3f(_sum * (1./d));
    return HomogeneousPoint3f::Zero();
  }

  inline Eigen::Matrix4f covariance() const { 
    float d = _sum.coeff(3,0);
    if (d){
      d=1./d;
      Eigen::Vector4f mean_=_sum*d;
      return (_squaredSum )*d - mean_*mean_.transpose();
    }
    return Eigen::Matrix4f::Zero(); 
  }

  inline const Eigen::Vector4f& sum() const {return _sum;}
  inline const Eigen::Matrix4f& squaredSum() const {return _squaredSum;}
  inline int n() const { return _sum.coeff(3,0);}
protected:
  Eigen::Vector4f _sum;
  Eigen::Matrix4f  _squaredSum;
};

#endif
