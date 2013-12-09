#pragma once

#include "homogeneousvector4f.h"

namespace pwn {

  struct InformationMatrix : public Eigen::Matrix4f {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    inline InformationMatrix() : Eigen::Matrix4f() { setZero(); }
    virtual ~InformationMatrix() {}

    inline InformationMatrix(const Eigen::Matrix4f &other) : Eigen::Matrix4f(other) {
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
      this->Eigen::Matrix4f::operator=(other);
      block<1, 4>(3, 0).setZero();
      block<4, 1>(0, 3).setZero();
      return *this;
    }

    template<typename OtherDerived>
      inline InformationMatrix transform(const Eigen::MatrixBase<OtherDerived> &other) const {
      Eigen::Matrix4f T = other;
      T.block<1, 4>(3, 0).setZero();
      T.block<4, 1>(0, 3).setZero();
      InformationMatrix s(T*(*this)*T.transpose());
      return s;
    }

    template<typename OtherDerived>
      inline InformationMatrix& transformInPlace(const Eigen::MatrixBase<OtherDerived> &other) const {
      const Eigen::Matrix3f& R = other.block<3,3>(0,0);
      block<3, 3>(0, 0) = R * block<3, 3>(0, 0) * R.transpose(); 
      return *this;
    }
  };

  class InformationMatrixVector : public TransformableVector<InformationMatrix> {
  public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    inline InformationMatrixVector() : TransformableVector<InformationMatrix>() {}
    virtual ~InformationMatrixVector() {}

    template<typename OtherDerived>
      inline void transformInPlace(const OtherDerived &m) {
      Eigen::Matrix4f T = m;
      T.row(3).setZero();
      T.col(3).setZero();
      Eigen::Matrix4f Tt = T.transpose();
      InformationMatrix *t = &(*this)[0];
      for (size_t i = 0; i < size(); i++, t++) {
	*t = T * (*t) * Tt;
      }
    }
  };

  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  typedef std::vector<Matrix6f, Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

}
