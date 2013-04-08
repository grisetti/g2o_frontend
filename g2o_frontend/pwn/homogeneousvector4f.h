#ifndef _HOMOGENEOUS_VECTOR_4F_H_
#define _HOMOGENEOUS_VECTOR_4F_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "transformable_vector.h"

template <int wCoordinate_>
struct HomogeneousVector4f: public Eigen::Vector4f {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static const float wCoordinate = wCoordinate_;
  typedef Eigen::Vector4f Base;

  inline HomogeneousVector4f() : Eigen::Vector4f(){
    this->data()[3]=wCoordinate;
  }

  inline HomogeneousVector4f(const Eigen::Vector3f& other){
    this->data()[0]=other.data()[0];
    this->data()[1]=other.data()[1];
    this->data()[2]=other.data()[2];
    this->data()[3]=wCoordinate;
  }

  template<typename OtherDerived>
  inline HomogeneousVector4f(const Eigen::MatrixBase<OtherDerived>& other)
    :Eigen::Vector4f(other){
    this->data()[3]=wCoordinate;
  }

  template<typename OtherDerived>
  inline HomogeneousVector4f& operator = (const Eigen::MatrixBase<OtherDerived>& other) {
    this->Base::operator=(other);
    this->data()[3]=wCoordinate;
    return *this;
  }

  inline HomogeneousVector4f& operator = (const Eigen::Vector3f& other) {
    this->data()[0]=other.data()[0];
    this->data()[1]=other.data()[1];
    this->data()[2]=other.data()[2];
    this->data()[3]=wCoordinate;
    return *this;
  }

  template<typename OtherDerived>
  inline HomogeneousVector4f transform(const Eigen::MatrixBase<OtherDerived>& other) const {
    return other*(*this);
  }

  template<typename OtherDerived>
  inline HomogeneousVector4f& transformInPlace(const Eigen::MatrixBase<OtherDerived>& other) const {
    *this = HomogeneousVector4f(other*(*this));
    return *this;
  }

};


typedef HomogeneousVector4f<1> HomogeneousPoint3f;

typedef HomogeneousVector4f<0> HomogeneousNormal3f;

typedef TransformableVector<HomogeneousPoint3f> HomogeneousPoint3fVector;

typedef TransformableVector<HomogeneousNormal3f> HomogeneousNormal3fVector;

#endif
