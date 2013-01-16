#include <cmath>
#include <algorithm>

template <int N, typename Base>
template <typename Base2>
_Vector<N,Base>::_Vector(const _Vector<N, Base2>& v){
  for (int i=0; i<size(); i++)
    _allocator[i]=Base(0.);
  int s=std::min(size(),v.size());
  for (int i=0; i<s; i++)
    _allocator[i]=v._allocator[i];
}

template <int N, typename Base>
_Vector<N,Base> _Vector<N,Base>::operator + (const _Vector<N,Base>& v) const {
  _Vector<N,Base> ret(*this);
  ret+=v;
  return ret;
};

template <int N, typename Base>
_Vector<N,Base>& _Vector<N,Base>::operator += (const _Vector<N,Base>& v) {
  for (int i=0; i<size(); i++)
    _allocator[i]+=v._allocator[i];
  return *this;
};

template <int N, typename Base>
_Vector<N,Base> _Vector<N,Base>::operator - (const _Vector<N,Base>& v) const {
  _Vector<N,Base> ret(*this);
  ret-=v;
  return ret;
};

template <int N, typename Base>
_Vector<N,Base>& _Vector<N,Base>::operator -= (const _Vector<N,Base>& v) {
  for (int i=0; i<size(); i++)
    _allocator[i]-=v._allocator[i];
  return *this;
};

template <int N, typename Base>
Base _Vector<N,Base>::operator *(const _Vector<N,Base>& v) const{
  Base acc=Base(0);
  for (int i=0; i<size(); i++)
    acc+=_allocator[i]*v._allocator[i];
  return acc;
}

template <int N, typename Base>
_Vector<N,Base>& _Vector<N,Base>::operator *= (Base c){
  for (int i=0; i<size(); i++)
    _allocator[i]*=c;
  return *this;
}

template <int N, typename Base>
_Vector<N,Base> _Vector<N,Base>::operator * (Base c) const{
  _Vector<N,Base> ret(*this);
  ret*=c;
  return ret;
}

template <int N, typename Base>
bool _Vector<N,Base>::operator==(const _Vector<N, Base>& other) const {
  for (int i=0; i<size(); ++i)
    if ((*this)[i]!=other[i]) return false;
  return true;
}

template <int N, typename Base>
Base _Vector<N,Base>::squaredNorm() const{
  Base acc=Base(0);
  for (int i=0; i<size(); i++)
    acc+=_allocator[i]*_allocator[i];
  return acc;
}

template <int N, typename Base>
Base _Vector<N,Base>::norm() const{
  return sqrt(squaredNorm());
}


template <int N, typename Base>
void _Vector<N, Base>::normalize(){
  Base f=squaredNorm();
  if (f>Base(0))
    (*this)*=sqrt(Base(1.)/f);
  else {
    (*this)-=(*this);
    _allocator[0]=Base(1.);
  }
}

template <int N, typename Base>
_Vector<N,Base> _Vector<N, Base>::normalized() const{
  _Vector<N,Base> aux(*this);
  aux.normalize();
  return aux;
}

template <int N, typename Base>
std::ostream& operator << (std::ostream& os, const _Vector<N, Base>& v){
  for (int j=0; j<v.size(); j++){
    if (j > 0)
      os << " ";
    os << v[j];
  }
  return os;
}

template <int N, typename Base>
void _Vector<N, Base>::fill(Base scalar)
{
  for (int i=0; i<size(); i++)
    _allocator[i] = scalar;
}

template <int N, typename Base>
_Vector<N, Base> operator* (Base x, const _Vector<N, Base>& v)
{
  return v * x;
}

