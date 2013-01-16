#ifndef _VECTORN_H_
#define _VECTORN_H_
#include <assert.h>
#include <iostream>
#include <stuff/array_allocator.h>

/** @addtogroup math **/
//@{

template <int Rows, int Cols, typename Base>
  struct _Matrix;

/**
This class represents an n-dimensional vector and implements the basic operations on vectors.
There are two memory models involved: fixed size and variable size.
The memory model is implemented by a member of type _ArrayAllocator.
Whenever it is possible use the fixed size memory model because the compiler can do
nice things when unrolling loops.

You can delclare a vector of fixed size by writing _Vector<X,T> when
X!=0 and T is tye type of the vecotr's element.  

A vector of variable size can be declared as _Vector<0,T>.

Vectors having different memory models are effectively *different* types,
and performing operations which will result in an illegal size results 
in a compilation error.

*/
template <int N, typename Base=double>
struct _Vector{
  typedef Base BaseType;
  typedef Base value_type; // compatibility with STL

  /**returns the size of a vector*/

  inline int size() const {return _allocator.size();}

  /**returns the ith element of a vector*/

  inline const Base& operator[](int i) const {return _allocator[i];}
  inline bool operator==(const _Vector<N, Base>& other) const;

  /**returns the ith element of a vector*/
  inline Base& operator[](int i) {return _allocator[i];}

  /**constructs a vector of size N. For fized size vectors N should not be specified*/
  _Vector(int s=N): _allocator(s){}
  static const int TemplateSize=N;

  /**constructs a vector from a vector having a different, but convertible 
     cell type (i.e. float and double)
   */
  template <typename Base2>
    _Vector(const _Vector<N, Base2>& v);
  
  /**helper constructor for a 2d vector.
     @param x: v[0]
     @param y: v[1]
   */
  _Vector(Base v0, Base v1){_allocator[0]=v0; _allocator[1]=v1;}

  /**helper constructor for a 3d vector.
     @param x: v[0]
     @param y: v[1]
     @param z: v[2]
   */
  _Vector(Base v0, Base v1, Base v2) {_allocator[0]=v0; _allocator[1]=v1; _allocator[2]=v2;}

  /**helper constructor for a 4d vector (often used for quaternions).
     @param x: v[0]
     @param y: v[1]
     @param z: v[2]
     @param w: v[3]
   */
  _Vector(Base v0, Base v1, Base v2, Base v3) {_allocator[0]=v0; _allocator[1]=v1; _allocator[2]=v2; _allocator[3]=v3;}


  // convenience accessor functions for transformations
  /**returns the v[0]*/
  inline const Base& x()     const  {return _allocator[0];}

  /**returns the v[1]*/
  inline const Base& y()     const  {return _allocator[1];}

  /**returns the v[2]*/
  inline const Base& z()     const  {return _allocator[2];}

  /**returns the v[3]*/
  inline const Base& w()     const  {return _allocator[3];}

  /**returns the v[0], which can be modified*/
  inline Base& x()            {return _allocator[0];}

  /**returns the v[1], which can be modified*/
  inline Base& y()            {return _allocator[1];}

  /**returns the v[2], which can be modified*/
  inline Base& z()            {return _allocator[2];}

  /**returns the v[3], which can be modified*/
  inline Base& w()            {return _allocator[3];}

  // rotational view
  /**returns the v[0], when the vector represents a rotation in euler angles*/
  inline const Base& roll()   const  {if (size() == 6) return _allocator[3]; return _allocator[0];}

  /**returns the v[1], when the vector represents a rotation in euler angles*/
  inline const Base& pitch() const  {if (size() == 6) return _allocator[4]; return _allocator[1];}

  /**returns the v[2], when the vector represents a rotation in euler angles*/
  inline const Base& yaw()   const  {if (size() == 6) return _allocator[5]; return _allocator[2];}

  /**returns the v[0], when the vector represents a rotation in euler angles*/
  inline Base& roll()          {if (size() == 6) return _allocator[3]; return _allocator[0];}

  /**returns the v[1], when the vector represents a rotation in euler angles*/
  inline Base& pitch()        {if (size() == 6) return
 _allocator[4]; return _allocator[1];}

  /**returns the v[2], when the vector represents a rotation in euler angles*/
  inline Base& yaw()          {if (size() == 6) return _allocator[5]; return _allocator[2];}


  /**vector sum
     @returns *this + v
   */
  _Vector<N,Base> operator + (const _Vector<N,Base>& v) const;

  /**vector accumulation. Always use v+=v2 instead of v=v+v2
     because is more efficient;
   */
  _Vector<N,Base>& operator += (const _Vector<N,Base>& v);

  /**vector difference
     @returns *this - v
   */
  _Vector<N,Base> operator - (const _Vector<N,Base>& v) const;

  /**vector cumulateve subtraction. The same considerations
     as for the cumulattive sum hold.
   */
  _Vector<N,Base>& operator -= (const _Vector<N,Base>& v);

  /**Dot product
   */
  Base operator *(const _Vector<N,Base>& v) const;

  /**Product by a scalar with side effect
   */
  _Vector<N,Base>& operator *= (Base c);

  /**Product by a scalar without side effect
   */
  _Vector<N,Base> operator * (Base c) const;

  /**returns the scalar product of a vector by itself
   */
  Base squaredNorm() const;

  /**vector norm
   */
  Base norm() const;

  /**scales the vector to have norm 1.
   */
  void normalize();

  /**sets all emements of a vector to the same value.
     @param scalar: the value
   */
  void fill(Base scalar);

  /**returns the normalized vector.
   */
  _Vector<N,Base> normalized() const;

  /**returns a column matrix representing the vector.
   */
  operator _Matrix<N, 1, Base>() const;

  /**this is the memory model.
   */
  _ArrayAllocator<N,Base> _allocator;

};

/** streaming operator
 */
template <int N, typename Base>
  std::ostream& operator << (std::ostream& os, const _Vector<N, Base>& v);

  /** product by a scalar, syntax scalar * vector */
template <int N, typename Base>
_Vector<N, Base> operator* (Base x, const _Vector<N, Base>& v);

typedef _Vector<2,int>   Vector2i;
typedef _Vector<2,float> Vector2f;
typedef _Vector<3,float> Vector3f;
typedef _Vector<6,float> Vector6f;


typedef _Vector<2,double> Vector2;
typedef _Vector<3,double> Vector3;
typedef _Vector<6,double> Vector6;

// dynamic size vectors
typedef _Vector<0,double> VectorX;
typedef _Vector<0,float>  VectorXf;

template <int N, typename Base>
  void st2dyn(_Vector<0,Base>& dest, const _Vector<N,Base> src){
  std2dyn(dest._allocator, src._allocator);
}

template <int N, typename Base>
  void dyn2st(_Vector<0,Base>& dest, const _Vector<N,Base> src){
  dyn2st(dest._allocator, src._alloator);
}

//@}

#include "vector_n.hpp"

#endif
