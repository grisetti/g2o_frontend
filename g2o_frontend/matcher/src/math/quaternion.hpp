#include <limits>
#include <algorithm>

template<typename Base>
_Quaternion<Base>::_Quaternion(): _Vector<4, Base>(Base(0), Base(0), Base(0), Base(1)){
}

template<typename Base>
_Quaternion<Base>::_Quaternion(const _Vector<4, Base>& v): _Vector<4, Base>(v){
}

template<typename Base>
_Quaternion<Base>::_Quaternion(Base _x, Base _y, Base _z, Base _w): _Vector<4, Base>(_x,_y,_z,_w){
}

template<typename Base>
_Quaternion<Base>::_Quaternion(const _RotationMatrix3<Base>& _r){
  this->x()=_r[2][1]-_r[1][2];
  this->y()=_r[0][2]-_r[2][0];
  this->z()=_r[1][0]-_r[0][1];
  this->w()=Base(1.)+_r[0][0]+_r[1][1]+_r[2][2];
  this->normalize();
}

template<typename Base>
_Quaternion<Base>::_Quaternion(const _Vector<3, Base>& vec){
  *this=_Quaternion(_RotationMatrix3<Base>(vec));
}

template<typename Base>
_Quaternion<Base>::_Quaternion(Base roll, Base pitch, Base yaw){
  *this=_Quaternion(_RotationMatrix3<Base>(roll, pitch, yaw));
}
 
template<typename Base>
inline _Quaternion<Base> _Quaternion<Base>::inverse() const{
  return _Quaternion<Base>(-this->x(), -this->y(), -this->z(), this->w());
}

template<typename Base>
inline _Quaternion<Base>& _Quaternion<Base>::normalize(){
  Base n = this->w()>0 ? this->norm() : -this->norm();
  if (fabs(n) > 1e-9){
    this->_Vector<4, Base>::operator*=(Base(1.)/n);
  }
  else
    *this= _Quaternion<Base>(0.,0.,0.,1.);
  return *this;
}

template<typename Base>
inline _Quaternion<Base> _Quaternion<Base>::normalized() const{
  _Quaternion<Base> q(*this);
  return q.normalize();
}


template<typename Base>
inline _Vector<3, Base> _Quaternion<Base>::operator * (const _Vector<3, Base>& point) const{
  const double sa =   point[0]*this->x() + point[1]*this->y() + point[2]*this->z();
  const double sb =   point[0]*this->w() - point[1]*this->z() + point[2]*this->y();
  const double sc =   point[0]*this->z() + point[1]*this->w() - point[2]*this->x();
  const double sd = - point[0]*this->y() + point[1]*this->x() + point[2]*this->w();
  _Vector<3, Base> out;
  out[0] = this->w() * sb + this->x() * sa + this->y() * sd - this->z() * sc;
  out[1] = this->w() * sc - this->x() * sd + this->y() * sa + this->z() * sb;
  out[2] = this->w() * sd + this->x() * sc - this->y() * sb + this->z() * sa;
  return out;
}

template<typename Base>
inline _Quaternion<Base> _Quaternion<Base>::operator * (const _Quaternion<Base>& q2) const{
  _Quaternion<Base> q(this->y()*q2.z() - q2.y()*this->z() + this->w()*q2.x() + q2.w()*this->x(),
		     this->z()*q2.x() - q2.z()*this->x() + this->w()*q2.y() + q2.w()*this->y(),
		     this->x()*q2.y() - q2.x()*this->y() + this->w()*q2.z() + q2.w()*this->z(),
		     this->w()*q2.w() - this->x()*q2.x() - this->y()*q2.y() - this->z()*q2.z());
  //  return q.normalize();
  return q.normalize();
  //return Quaternion(q.rotationMatrix());
}

template<typename Base>
inline _Quaternion<Base>& _Quaternion<Base>::operator *= (const _Quaternion<Base>& q2){
  _Quaternion<Base> q=(*this*q2);
  *this=q.normalize();
  return *this;
}


template<typename Base>
inline _Vector<3, Base> _Quaternion<Base>::angles() const
{
  return rotationMatrix().angles();
}

template<typename Base>
_RotationMatrix3<Base> _Quaternion<Base>::rotationMatrix() const 
{
  _RotationMatrix3<Base> rot;
  Base w2=this->w()*this->w();
  Base x2=this->x()*this->x();
  Base y2=this->y()*this->y();
  Base z2=this->z()*this->z();
  Base xy=this->x()*this->y();
  Base xz=this->x()*this->z();
  Base xw=this->x()*this->w();
  Base yz=this->y()*this->z();
  Base yw=this->y()*this->w();
  Base zw=this->z()*this->w();
  rot[0][0] = w2 + x2 - y2 - z2; rot[0][1] = 2.0 * (xy - zw);   rot[0][2] = 2.0 * (xz + yw);
  rot[1][0] = 2.0 * (xy + zw);   rot[1][1] = w2 - x2 + y2 - z2; rot[1][2] = 2.0 * (yz - xw);
  rot[2][0] = 2.0 * (xz - yw);   rot[2][1] = 2.0 * (yz + xw);   rot[2][2] = w2 - x2 - y2 + z2;
  return rot;
}

template<typename Base>
inline Base _Quaternion<Base>::angle() const{
  _Quaternion<Base> q=normalized();
  double a=2*atan2(sqrt(q.x()*q.x() + q.y()*q.y()  + q.z()*q.z()), q.w());
  return atan2(sin(a), cos(a));
}

template<typename Base>
inline _Quaternion<Base> _Quaternion<Base>::slerp(const _Quaternion<Base>& from, const _Quaternion<Base>& to, Base lambda){
  Base _cos_omega = _Vector<4, Base>(from)*_Vector<4, Base>(to);
	_cos_omega = (_cos_omega>1)?1:_cos_omega;
	_cos_omega = (_cos_omega<-1)?-1:_cos_omega;
	Base _omega = acos(_cos_omega);
	assert (!isnan(_cos_omega));
	if (fabs(_omega) < 1e-9)
		return to;
	
	//determine right direction of slerp:
	_Quaternion<Base> _pq = from - to;
	_Quaternion<Base> _pmq = from + to;
	Base _first = _pq.norm();
	Base _alternativ = _pmq.norm();
	
	_Quaternion<Base> q1 = from;
	_Quaternion<Base> q2 = to;

	if (_first > _alternativ)
	  q2._Vector<4, Base>::operator*=(Base(-1.));

	//now calculate intermediate quaternion.
	Base alpha=sin((1-lambda)*_omega)/(sin(_omega));
	Base beta=sin((lambda)*_omega)/(sin(_omega));
	_Quaternion<Base> ret((_Vector<4, Base>)q1*(alpha) + (_Vector<4, Base>)q2*(beta));
	assert (!(isnan(ret.w()) || isnan(ret.x()) || isnan(ret.y()) || isnan(ret.z())));
	return ret.normalized();
}

template<typename Base>
template<typename Base2>
_Quaternion<Base>::_Quaternion(const _Quaternion<Base2>& other){
  (*this)[0] = other[0];
  (*this)[1] = other[1];
  (*this)[2] = other[2];
  (*this)[3] = other[3];
}
