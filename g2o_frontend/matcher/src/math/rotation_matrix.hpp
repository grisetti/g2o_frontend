
template <int N, typename Base>
_RotationMatrix<N, Base>::_RotationMatrix(){
  _Matrix<N, N, Base> *m =this;
  *m=_Matrix<N, N, Base>::eye(1.0);
}


template <int N, typename Base>
_RotationMatrix<N, Base>::_RotationMatrix(const _Matrix<N, N, Base> &m){
  _Matrix<N, N, Base>* t=this;
  *t=m;
}

template <int N, typename Base>
_RotationMatrix<N, Base>& _RotationMatrix<N, Base>::operator*=(const _RotationMatrix<N, Base>& m){
  _Matrix<N,N, Base>::operator*=(m);
  return *this;
}

template <int N, typename Base>
_RotationMatrix<N, Base> _RotationMatrix<N, Base>::operator*(const _RotationMatrix<N, Base>& m) const{
  _RotationMatrix t(*this);
  t*=m;
  return t;
}

template <int N, typename Base>
_Vector<N, Base> _RotationMatrix<N, Base>::operator*(const _Vector<N, Base>& v) const{
  return _Matrix<N,N,Base>::operator*(v);
}

template <int N, typename Base>
_RotationMatrix<N, Base> _RotationMatrix<N, Base>::inverse() const{
  _RotationMatrix t=(*this);
  t.transposeInPlace();
  return t;
}


template <typename Base>
_RotationMatrix2<Base>::_RotationMatrix2(Base angle){
  Base c=cos(angle), s=sin(angle);
  this->_allocator[0][0] = c;  
  this->_allocator[0][1] = -s; 
  this->_allocator[1][0] = s;  
  this->_allocator[1][1] = c; 
}

template <typename Base>
_RotationMatrix2<Base>::_RotationMatrix2(const _Vector<1,Base>& a) {
  Base c=cos(a[0]), s=sin(a[0]);
  this->_allocator[0][0] = c;  
  this->_allocator[0][1] = -s; 
  this->_allocator[1][0] = s;  
  this->_allocator[1][1] = c; 
}

template <typename Base>
Base _RotationMatrix2<Base>::angle() const{
  return atan2(this->_allocator[1][0], this->_allocator[0][0]);
}


template <typename Base>
_RotationMatrix3<Base>::_RotationMatrix3(Base roll, Base pitch, Base yaw){
    Base sphi   = sin(roll);
    Base stheta = sin(pitch);
    Base spsi   = sin(yaw);
    Base cphi   = cos(roll);
    Base ctheta = cos(pitch);
    Base cpsi   = cos(yaw);
	
    Base _r[3][3] = { //create rotational Matrix
      {cpsi*ctheta, cpsi*stheta*sphi - spsi*cphi, cpsi*stheta*cphi + spsi*sphi},
      {spsi*ctheta, spsi*stheta*sphi + cpsi*cphi, spsi*stheta*cphi - cpsi*sphi},
      {    -stheta,                  ctheta*sphi,                  ctheta*cphi}
    };
    for (int i=0; i<3; i++)
      for (int j=0; j<3; j++)
	this->_allocator[i][j]=_r[i][j];

}

template <typename Base>
_RotationMatrix3<Base>::_RotationMatrix3(const _Vector<3, Base>& angles) {
  *this=_RotationMatrix3(angles.roll(), angles.pitch(), angles.yaw());
}

template <typename Base>
_RotationMatrix3<Base>::_RotationMatrix3():_RotationMatrix<3, Base>() {
}

template <typename Base>
_RotationMatrix2<Base>::_RotationMatrix2():_RotationMatrix<2, Base>() {
}


template <typename Base>
_Vector<3, Base> _RotationMatrix3<Base>::angles() const{
  _Vector<3, Base> aux;
  aux.roll() = atan2(this->_allocator[2][1],this->_allocator[2][2]);
  aux.pitch() = atan2(-this->_allocator[2][0],sqrt(this->_allocator[2][1]*this->_allocator[2][1] + this->_allocator[2][2]* this->_allocator[2][2]));
  aux.yaw() = atan2(this->_allocator[1][0],this->_allocator[0][0]);
  return aux;
}
