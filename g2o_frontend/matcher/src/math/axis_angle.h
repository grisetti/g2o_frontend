#ifndef _AXIS_ANGLE_H_
#define _AXIS_ANGLE_H_
#include "quaternion.h"

/** @addtogroup math **/
//@{

/**Rotation as the rotation axis whose lenght is proportional 
   to the entity of the rotation. 
   The same convention as for al rotation classes hold.*/
template <typename Base=double>
struct _AxisAngle : public _Vector<3, Base>
{
  static const int Dimension=3;
  static const int Angles=3;
  _AxisAngle();
  _AxisAngle(const _Quaternion<Base>& q);
  _AxisAngle(const _RotationMatrix3<Base>& m);
  _AxisAngle(const _Vector<3, Base>& vec);
  _AxisAngle(Base roll, Base pitch, Base yaw);
  _AxisAngle(const _Vector<3, Base>& axis, Base angle);
  _AxisAngle<Base>& operator*=(const _AxisAngle& a);
  _AxisAngle<Base>  operator* (const _AxisAngle& a) const;
  _Vector<3, Base> operator*(const _Vector<3, Base>& v) const;
  inline _AxisAngle<Base> inverse() const;
  inline _Vector<3, Base> angles() const;
  _RotationMatrix3<Base> rotationMatrix() const;
  _Quaternion<Base> quaternion() const;
};

typedef _AxisAngle<double> AxisAngle;
typedef _AxisAngle<float> AxisAnglef;

//@}

#include "axis_angle.hpp"

#endif
