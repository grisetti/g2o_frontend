#ifndef _HOMOGENEOUSPOINT3FSTATS_H_
#define _HOMOGENEOUSPOINT3FSTATS_H_
#include "homogeneousvector4f.h"

struct HomogeneousPoint3fStats: public Eigen::Matrix4f{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  inline HomogeneousPoint3fStats() {
    setZero();
  }

  typedef Eigen::Matrix4f Base;

  template<typename OtherDerived>
  inline HomogeneousPoint3fStats(const Eigen::MatrixBase<OtherDerived>& other)
    :Eigen::Matrix4f(other){
    row(4).setZero();
  }

  template<typename OtherDerived>
  inline HomogeneousPoint3fStats& operator = (const Eigen::MatrixBase<OtherDerived>& other) {
    this->Base::operator=(other);
    row(4).setZero();
    return *this;
  }
  inline Eigen::Vector3f eigenValues() {return block<3,1>(0,3);}
  inline Eigen::Matrix3f eigenVectors(){return block<3,3>(0,0);};
  inline float curvature() const {return coeffRef(0,3)/(coeffRef(0,3)+coeffRef(1,3)+coeffRef(2,3));}
};

typedef TransformableVector<HomogeneousPoint3fStats> HomogeneousPoint3fStatsVector;

#endif
