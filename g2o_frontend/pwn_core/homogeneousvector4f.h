#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "../basemath/transformable_vector.h"

namespace pwn {

  template <int wCoordinate_>
    class HomogeneousVector4f: public Eigen::Vector4f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const float wCoordinate = wCoordinate_;

    inline HomogeneousVector4f() : Eigen::Vector4f() { this->data()[3] = wCoordinate; }

    inline HomogeneousVector4f(const Eigen::Vector3f &other) {
      this->data()[0] = other.data()[0];
      this->data()[1] = other.data()[1];
      this->data()[2] = other.data()[2];
      this->data()[3] = wCoordinate;
    }

    template<typename OtherDerived>
      inline HomogeneousVector4f(const Eigen::MatrixBase<OtherDerived> &other) : Eigen::Vector4f(other) { this->data()[3] = wCoordinate; }

    template<typename OtherDerived>
      inline HomogeneousVector4f& operator = (const Eigen::MatrixBase<OtherDerived> &other) {
      this->Eigen::Vector4f::operator = (other);
      this->data()[3] = wCoordinate;
      return *this;
    }

    inline HomogeneousVector4f& operator = (const Eigen::Vector3f &other) {
      this->data()[0] = other.data()[0];
      this->data()[1] = other.data()[1];
      this->data()[2] = other.data()[2];
      this->data()[3] = wCoordinate;
      return *this;
    }

  };

  typedef HomogeneousVector4f<1> Point;
  typedef HomogeneousVector4f<0> Normal;
  typedef TransformableVector<Point> PointVector;
  typedef TransformableVector<Normal> NormalVector;

}
