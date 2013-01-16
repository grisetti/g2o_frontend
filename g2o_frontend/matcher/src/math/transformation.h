#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_
#include "rotation_matrix.h"
#include "angle.h"
#include "quaternion.h"
#include "axis_angle.h"

/** @addtogroup math **/
//@{

/**Implements an transformation class. A transformation is parametrized by the 
RotationType. The translation type is inferred by the dimension of the rotation,
and it is always a vector.
Rotations can be compose with the * operator.
The inverse is obtained with the inverse() method.
A rotation R can be applied to a vector v by the * operator R*v.
*/ 
template <typename Rotation>
struct _Transformation {
  typedef typename Rotation::BaseType BaseType;
  typedef Rotation RotationType;
  typedef _Vector<Rotation::Dimension, BaseType> TranslationType;
  typedef _Matrix<Rotation::Dimension+1, Rotation::Dimension+1, BaseType> TransformationMatrix;
  typedef _Vector<Rotation::Dimension+Rotation::Angles, BaseType> TransformationVector;
  
  _Transformation() {
    _translation.fill(0.0);//=TranslationType()*BaseType(0.);
  }

  _Transformation(const TranslationType& t, const Rotation& r) {
    _translation=t;
    _rotation=r;
  }

  TranslationType& translation() {return _translation;}

  const TranslationType& translation() const {return _translation;}

  Rotation& rotation() {return _rotation;}

  const Rotation& rotation() const {return _rotation;}

  _Transformation<Rotation> operator  * (const _Transformation<Rotation>& t) const {
    return _Transformation<Rotation>(translation()+rotation()*t.translation(), rotation()*t.rotation());
  }

  _Transformation<Rotation>& operator *= (const _Transformation<Rotation>& t) {
    _translation+=rotation()*t.translation();
    _rotation*=t.rotation();
    return *this;
  }
  
  _Transformation<Rotation> inverse() const {
    RotationType _inverse=_rotation.inverse();
    return _Transformation<Rotation>(_inverse*(_translation*BaseType(-1.)), _inverse);
  }

  TranslationType operator* (const TranslationType& t) const {
    return TranslationType(translation()+rotation()*t);
  }

  TransformationMatrix toMatrix() const {
    TransformationMatrix m= TransformationMatrix::eye(1.);
    _Matrix <Rotation::Dimension, Rotation::Dimension, BaseType> r = _rotation.rotationMatrix();
    for (int i=0; i<r.rows(); i++)
      for (int j=0; j<r.cols(); j++)
	m[i][j]=r[i][j];
    for (int i=0; i<_translation.size(); i++)
      m[i][m.cols()-1]=_translation[i];
    return m;
  }

  TransformationVector toVector() const {
    TransformationVector v;
    for (int i=0; i<_translation.size(); i++)
      v[i]=_translation[i];
    _Vector<RotationType::Angles, BaseType> a=_rotation.angles();
    for (int i=0; i<a.size(); i++)
      v[i+RotationType::Dimension]=a[i];
    return v;
  }

  static _Transformation<Rotation> fromVector(const TransformationVector& v) {
    TranslationType translation_;
    _Vector<RotationType::Angles, BaseType> a;
    for (int i=0; i<RotationType::Dimension; i++)
      translation_[i]=v[i];
    for (int i=0; i<a.size(); i++)
      a[i]=v[i+RotationType::Dimension];
    RotationType rotation_(a);
    return _Transformation<Rotation>(translation_,rotation_);
  }

  BaseType operator[](int i) const {
    if (i<Rotation::Dimension){
      return _translation[i];
    } else {
      return _rotation[i-Rotation::Dimension];
    }
  } 

  BaseType& operator[](int i) {
    if (i<Rotation::Dimension){
      return _translation[i];
    } else {
      return _rotation[i-Rotation::Dimension];
    }
  } 



  TranslationType _translation;
  RotationType    _rotation;
};

typedef _Transformation< RotationMatrix2f> _Transformation2rf;
typedef _Transformation< Anglef> _Transformation2f;
typedef _Transformation< RotationMatrix2> _Transformation2r;
typedef _Transformation< Angle> _Transformation2;

typedef _Transformation< RotationMatrix3f> Transformation3rf;
typedef _Transformation< Quaternionf>      Transformation3f;
typedef _Transformation< AxisAnglef>       Transformation3af;

typedef _Transformation< Quaternion>      Transformation3;
typedef _Transformation< RotationMatrix3> Transformation3r;
typedef _Transformation< AxisAngle>       Transformation3a;

typedef _Transformation< Angle> Transformation2;

template <typename Rotation>
ostream& operator<<(ostream& os, const _Transformation<Rotation>& t)
{
  os << t.toVector();
  return os;
}

//@}

// creation methods for transformation2



template <typename Base>
_Transformation< _Quaternion<Base>  > manifold2Transformation(const typename _Transformation< _Quaternion<Base>  >::TransformationVector& v) {
  _Vector<3, Base> t;
  t[0] = v[0];
  t[1] = v[1];
  t[2] = v[2];
  _AxisAngle<Base> a;
  a[0]=v[3];
  a[1]=v[4];
  a[2]=v[5];
  return _Transformation< _Quaternion<Base>  >(t,a.quaternion());
}

template <typename Base> 
typename _Transformation< _Quaternion<Base>  >::TransformationVector transformation2manifold(const _Transformation< _Quaternion<Base>  >& t) {
  typename _Transformation< _Quaternion<Base>  >::TransformationVector v;
  v[0] = t.transformation()[0];
  v[1] = t.transformation()[1];
  v[2] = t.transformation()[2];
  _AxisAngle<Base> a(t.rotation());
  v[3]=a[3];
  v[4]=a[4];
  v[5]=a[5];
  return v;
}



#endif
