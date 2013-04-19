#ifndef _HOMOGENEOUSPOINT3FOMEGA_H_
#define _HOMOGENEOUSPOINT3FOMEGA_H_

#include "homogeneousvector4f.h"
#include <iostream>

struct HomogeneousPoint3fOmega: public Eigen::Matrix4f{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef Eigen::Matrix4f Base;

  inline HomogeneousPoint3fOmega() { setZero(); }

  template<typename OtherDerived>
  inline HomogeneousPoint3fOmega(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix4f(other) { 
    block<1,4>(3,0).setZero();
    block<4,1>(0,3).setZero();
  }

  template<typename OtherDerived>
    inline HomogeneousPoint3fOmega& operator = (const Eigen::MatrixBase<OtherDerived>& other) {
    this->Base::operator=(other);
    block<1,4>(3,0).setZero();
    block<4,1>(0,3).setZero();
    return *this;
  }

  template<typename OtherDerived>
  inline HomogeneousPoint3fOmega transform(const Eigen::MatrixBase<OtherDerived>& other) const {
    HomogeneousPoint3fOmega s(other*(*this)*other.transpose());
    return s;
  }

  template<typename OtherDerived>
  inline HomogeneousPoint3fOmega& transformInPlace(const Eigen::MatrixBase<OtherDerived>& other) const {
    HomogeneousPoint3fOmega s(other*(*this)*other.transpose());
    *this = s;
    return *this;
  }
};

typedef Eigen::Matrix<float, 6, 6> Matrix6f;

typedef std::vector<Matrix6f, Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

typedef TransformableVector<HomogeneousPoint3fOmega> HomogeneousPoint3fOmegaVector;

#endif
