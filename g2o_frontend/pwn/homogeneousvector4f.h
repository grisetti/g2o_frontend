/** \file homogeneousvector4f.h
 *  \brief HomogeneousVector4f struct header file.
 *  This is the header file of the HomogeneousVector4f struct.
*/

#ifndef _HOMOGENEOUS_VECTOR_4F_H_
#define _HOMOGENEOUS_VECTOR_4F_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "transformable_vector.h"

/** \struct HomogeneousVector4f
 *  \brief Base class used for 4 elements homogeneous vectors.
 *
 *  This class allows to create homogeneous vectors of 4 elements sepcifying
 *  the integer value to assign to the fourth coordinate. It extends the Eigen Vector4f
 *  class.
 */

template <int wCoordinate_>
struct HomogeneousVector4f: public Eigen::Vector4f {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   *  Variable containing the fourth coordinate of the HomogeneousVector4f object.
   *  It represents the homogeneous coordinate value.
   */
  static const float wCoordinate = wCoordinate_;
  
  /** \typedef Base
   *  \brief 4 elements float vector type.
   *
   *  The Base type it's an Eigen Vector4f.
   */
  typedef Eigen::Vector4f Base;

  /**
   *  Empty constructor.
   *  This constructor creates an HomogeneousVector4f object using the empty constructor of 
   *  the Eigen Vector4f class and imposing the fourth coordinate to be equal to the integer
   *  value specified inside the template.
   */
  inline HomogeneousVector4f() : Eigen::Vector4f() {
    this->data()[3] = wCoordinate;
  }

  /**
   *  Constructor using an Eigen Vector3f object.
   *  This constructor creates a HomogeneousVector4f object using the values of the Eigen
   *  Vector3f given in input for the the first three coordinates. The fourth coordinate 
   *  is setted to be equal to the integer value specified inside the template.
   *  @param other is the 3 elements vector used to fill the first three coordinates of the
   *  HomogeneousVector4f object.
   */
  inline HomogeneousVector4f(const Eigen::Vector3f& other) {
    this->data()[0] = other.data()[0];
    this->data()[1] = other.data()[1];
    this->data()[2] = other.data()[2];
    this->data()[3] = wCoordinate;
  }

  /**
   *  Constructor using an Eigen MatrixBase object.
   *  This constructor creates an HomogeneousVector4f object using the constructor of the
   *  Eigen Vector4f class using as input the given generic Eigen MatrixBase object. Then it impose 
   *  the fourth coordinate to be equal to the integer value specified inside the template.
   *  @param other is the Eigen MatrixBase object used to fill the first three coordinates of the
   *  HomogeneousVector4f object.
   */
  template<typename OtherDerived>
  inline HomogeneousVector4f(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Vector4f(other) {
    this->data()[3] = wCoordinate;
  }

  /**
   *  This method define the assignment operator between a generic Eigen
   *  MatrixBase object and an HomogeneousVector4f object.
   */
  template<typename OtherDerived>
  inline HomogeneousVector4f& operator = (const Eigen::MatrixBase<OtherDerived>& other) {
    this->Base::operator = (other);
    this->data()[3] = wCoordinate;
    return *this;
  }

  /**
   *  This method define the assignment operator between an Eigen
   *  Vector3f object and an HomogeneousVector4f object.
   */
  inline HomogeneousVector4f& operator = (const Eigen::Vector3f& other) {
    this->data()[0] = other.data()[0];
    this->data()[1] = other.data()[1];
    this->data()[2] = other.data()[2];
    this->data()[3] = wCoordinate;
    return *this;
  }

};

/** \typedef HomogeneousPoint3f
 *  \brief 4 elements float vector type for homogeneous 3D point representation.
 *
 *  The HomogeneousPoint3f type it's an HomogeneousVector4f where the fourth coordinates it's setted
 *  to one.
*/
typedef HomogeneousVector4f<1> HomogeneousPoint3f;

/** \typedef HomogeneousNormal3f
 *  \brief 4 elements float vector type for homogeneous 3D point normal representation.
 *
 *  The HomogeneousNormal3f type it's an HomogeneousVector4f where the fourth coordinates it's setted
 *  to zero.
*/
typedef HomogeneousVector4f<0> HomogeneousNormal3f;

/** \typedef HomogeneousPoint3fVector
 *  \brief TransformableVector of HomogeneousPoint3f.
 *
 *  The HomogeneousPoint3fVector type it's a TrasformableVector of 3D points expressed in
 *  homogeneous coordinates.
*/
typedef TransformableVector<HomogeneousPoint3f> HomogeneousPoint3fVector;

/** \typedef HomogeneousNormal3fVector
 *  \brief TransformableVector of HomogeneousNormal3f.
 *
 *  The HomogeneousNormal3fVector type it's a TrasformableVector of 3D points normals expressed in
 *  homogeneous coordinates.
*/
typedef TransformableVector<HomogeneousNormal3f> HomogeneousNormal3fVector;

#endif
