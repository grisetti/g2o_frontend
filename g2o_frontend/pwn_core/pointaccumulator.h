#pragma once

#include "pwn_typedefs.h"
#include "homogeneousvector4f.h"

namespace pwn {

  class PointAccumulator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    inline PointAccumulator() { clear(); }
    virtual ~PointAccumulator() {}
  
    inline void clear() {
      _sum.setZero();
      _squaredSum.setZero();
    }
  
    inline void operator +=(const PointAccumulator &pa) {
      _sum += pa._sum;
      _squaredSum += pa._squaredSum;
    }
  
    inline void operator -=(const PointAccumulator &pa) {
      _sum -= pa._sum;
      _squaredSum -= pa._squaredSum;
    }

    inline void operator +=(const Point &v) {
      _sum += (Eigen::Vector4f&)v;
      _squaredSum += v * v.transpose();
    }

    inline Point mean() const { 
      const float &d = _sum.coeff(3, 0);
      if(d)
	return Point(_sum * (1.0f / d));
      return Point::Zero();
    }

    inline Eigen::Matrix4f covariance() const { 
      float d = _sum.coeff(3, 0);
      if(d) {
	d = 1.0f / d;
	Eigen::Vector4f mean_ = _sum * d;
	return _squaredSum * d - mean_ * mean_.transpose();
      }
      return Eigen::Matrix4f::Zero(); 
    }

    inline const Eigen::Vector4f& sum() const { return _sum; }
  
    inline const Eigen::Matrix4f& squaredSum() const { return _squaredSum; }
  
    inline int n() const { return _sum.coeff(3, 0); }

  protected:
    Eigen::Vector4f _sum;
    Eigen::Matrix4f  _squaredSum;
  };

}

