#ifndef _ROTATIONMATRIX_H_
#define _ROTATIONMATRIX_H_

#include "matrix_n.h"

/** @addtogroup math **/
//@{
/** Implements a generic n-dimensonal rotation matrix.
    A rotation matrix can be multiplied by another rotation matrix, 
    leading to the composition of the rotations.
    It can be inverted (which means transposed), resulting 
    in the inverse rotation.
    It can be miltiplied by a vector, leading to the rotated vector.
*/
template <int N, typename Base=double> 
  struct _RotationMatrix: public _Matrix<N, N, Base> {
  static const int Dimension=N;
  /**Constructs a rotation matrix represeting the zero rotation*/
  _RotationMatrix();
  /**applies to this rotation matrix the rotation passed as argument*/
  _RotationMatrix<N,Base>& operator*=(const _RotationMatrix<N,Base>& m);
  /**returns the composition of rotations*/
  _RotationMatrix<N,Base> operator*(const _RotationMatrix<N,Base>& m) const;
  /**returns the rotated vector*/
  _Vector<N,Base> operator*(const _Vector<N,Base>& v) const;
  /**returns the rotated inverse*/
  _RotationMatrix<N,Base> inverse() const;
 protected:
  _RotationMatrix(const _Matrix<N,N,Base> &m);
};

/**Specialized class for handling 2D rotations as rotation matrices*/
template < typename Base=double >
struct _RotationMatrix2: public _RotationMatrix<2, Base>{
  static const int Angles=1;
  _RotationMatrix2();
  /**constructs a rotation matric from an angle value*/
  _RotationMatrix2(Base angle);
  /**constructs a rotation matric from a 3 vector representing the euler angle*/
  _RotationMatrix2(const _Vector<1,Base>& angles);
  _RotationMatrix2(const _RotationMatrix<2,Base>& rm) {this->_allocator=rm._allocator;}
  /**returns a rotation matrix which is a copy, 
     Implemented be homogeneous with other rotation representations*/
  _RotationMatrix2<Base> rotationMatrix() const {return *this;}
  /**returns the euler angles*/
  _Vector<1, Base> angles() const {_Vector<1, Base> v; v[0]=angle(); return v;}
  Base angle() const;
};

/**Specialized class for handling 3D rotations as rotation matrices*/
template <typename Base = double >
  struct _RotationMatrix3: public _RotationMatrix<3, Base>{
  static const int Angles=3;
  _RotationMatrix3();
  /**constructs a rotation matric from a 3 vector representing the euler angles*/
  _RotationMatrix3(const _Vector<3, Base>& angles);
  _RotationMatrix3(const _RotationMatrix<3,Base>& rm) {this->_allocator=rm._allocator;}
  /**constructs a rotation matric from a the euler angles*/
  _RotationMatrix3(Base roll, Base pitch, Base yaw);
  /**returns a rotation matrix which is a copy, 
     Implemented be homogeneous with other rotation representations*/
  _RotationMatrix3<Base> rotationMatrix() const {return *this;}
  /**returns the euler angles*/
  _Vector<3, Base> angles() const;
};
//@}

typedef _RotationMatrix2<double> RotationMatrix2;
typedef _RotationMatrix3<double> RotationMatrix3;
typedef _RotationMatrix2<float> RotationMatrix2f;
typedef _RotationMatrix3<float> RotationMatrix3f;

#include "rotation_matrix.hpp"


#endif
