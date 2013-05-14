#ifndef _INFORMATIONMATRIX_H_
#define _INFORMATIONMATRIX_H_

#include "homogeneousvector4f.h"

struct InformationMatrix: public Eigen::Matrix4f {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef Eigen::Matrix4f Base;


  inline InformationMatrix() : Base() { setZero(); }

  inline InformationMatrix(const Base &other) : Base(other) {
    block<1, 4>(3, 0).setZero();
    block<4, 1>(0, 3).setZero();
  }

  template<typename OtherDerived>
  inline InformationMatrix(const Eigen::MatrixBase<OtherDerived> &other) : Eigen::Matrix4f(other) {
    block<1, 4>(3, 0).setZero();
    block<4, 1>(0, 3).setZero();
  }

  template<typename OtherDerived>
    inline InformationMatrix& operator = (const Eigen::MatrixBase<OtherDerived> &other) {
    this->Base::operator=(other);
    block<1, 4>(3, 0).setZero();
    block<4, 1>(0, 3).setZero();
    return *this;
  }

  template<typename OtherDerived>
  inline InformationMatrix transform(const Eigen::MatrixBase<OtherDerived> &other) const {
    InformationMatrix s(other*(*this)*other.transpose());
    return s;
  }

  template<typename OtherDerived>
  inline InformationMatrix& transformInPlace(const Eigen::MatrixBase<OtherDerived> &other) const {
    InformationMatrix s(other*(*this)*other.transpose());
    *this = s;
    return *this;
  }
};

typedef Eigen::Matrix<float, 6, 6> Matrix6f;

typedef std::vector<Matrix6f, Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

typedef TransformableVector<InformationMatrix> InformationMatrixVector;

#endif
