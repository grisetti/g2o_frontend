#ifndef _INTEGRAL_POINT_IMAGE_H_
#define _INTEGRAL_POINT_IMAGE_H_
#include "pointwithnormal.h"

struct PointAccumulator{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PointAccumulator(){
    clear();
  }
  inline void clear() {
    _sum.setZero();
    _squaredSum.setZero();
    _n=0;
  }
  inline void operator +=(const PointAccumulator& pa) {
    _n+=pa._n;
    _sum+=pa._sum;
    _squaredSum+=pa._squaredSum;
  }
  inline void operator -=(const PointAccumulator& pa) {
    _n-=pa._n;
    _sum-=pa._sum;
    _squaredSum-=pa._squaredSum;
  }


  inline void operator +=(const Eigen::Vector3f& v) {
    _n++;
    _sum+=v;
    _squaredSum+=v*v.transpose();
  }

  inline Eigen::Vector3f mean() const { 
    if (_n)
      return _sum * (1./(float)_n);
    return Eigen::Vector3f::Zero();
  }

  inline Eigen::Matrix3f covariance() const { 
    if (_n){
      Eigen::Vector3f mean_=mean();
      return (_squaredSum )*(1./(float)_n) - mean_*mean_.transpose();
    }
    return Eigen::Matrix3f::Zero(); 
  }

  inline const Eigen::Vector3f& sum() const {return _sum;}
  inline const Eigen::Matrix3f& squaredSum() const {return _squaredSum;}
  inline int n() const {return _n;}

  int _n;

  Eigen::Vector3f _sum;
  Eigen::Matrix3f  _squaredSum;
};

struct IntegralPointImage: public Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic>{
  IntegralPointImage();
  void compute(const Eigen::MatrixXi pointIndices, const PointWithNormalVector& points);
  void clear();
  PointAccumulator getRegion(int xmin, int xmax, int ymin, int ymax) const;
};


#endif
