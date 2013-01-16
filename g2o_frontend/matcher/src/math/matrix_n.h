#ifndef _MATRIXN_H_
#define _MATRIXN_H_
#include <assert.h>
#include <iostream>
#include <limits>
#include "vector_n.h"

using namespace std;
/** @addtogroup math **/
//@{

/**This class implements a matrix. The meory model can be either static or dynamic.
Here the same considerations as for the vector class hold.

You can access the elemets of a matrix with A[i][j] in  avery nice way.
Multiply, sum and do anything with matrices pretty much as when you write atrix expressions.
All functions are more or less as one expects them in a regular matrix algebra.

Once again it is recommended to use the fixed size matrices whenever the size is known.
*/

template <int Rows, int Cols, typename Base=double>
  struct _Matrix{
    typedef Base BaseType;
    
    _Matrix(int r=Rows, int c=Cols):_allocator(r,c){}
    inline const Base* operator[](int row) const {return _allocator[row];}
    inline Base* operator[](int row) {return _allocator[row];}
    inline int cols() const {return _allocator.cols();}
    inline int rows() const {return _allocator.rows();}
    _Matrix<Rows, Cols, Base>& operator += (const _Matrix<Rows, Cols, Base>& v);
    _Matrix<Rows, Cols, Base>& operator -= (const _Matrix<Rows, Cols, Base>& v);
    _Matrix<Rows, Cols, Base> operator - (const _Matrix<Rows, Cols, Base>& v) const;
    _Matrix<Rows, Cols, Base> operator + (const _Matrix<Rows, Cols, Base>& v) const;
    _Matrix<Cols, Rows, Base> transpose() const;
    _Matrix<Cols, Rows, Base>& transposeInPlace();
    _Matrix<Rows, Cols, Base>& operator*=(const _Matrix<Rows, Cols, Base>& m);
    _Matrix<Rows, Cols, Base>& operator*=(Base c);
    _Matrix<Rows, Cols, Base> operator*(Base c) const;
    bool operator==(const _Matrix<Rows, Cols, Base>& other) const;

    template <int Rows1, int Cols1>
      _Matrix<Rows, Cols1, Base> operator*(const _Matrix<Rows1, Cols1, Base>& m) const;
    _Vector<Rows, Base> operator* (const _Vector<Cols, Base>& v) const;
    int nullSpace(_Matrix<Rows, Cols, Base>& nullS, Base epsilon=std::numeric_limits<Base>::epsilon() ) const;
    static _Matrix<Rows, Rows, Base> eye(Base factor, int dim=Rows);
    static _Matrix<Rows, Rows, Base> diag(const _Vector<Rows, Base>& v);
    static _Matrix<Rows, Rows, Base> outerProduct(const _Vector<Rows, Base>& v1, const _Vector<Rows, Base>& v2);
    static _Matrix<Rows, Rows, Base> permutation(const _Vector<Rows, int>& p);
    _Matrix<Rows, Rows, Base> inverse() const;
    _Matrix<Rows, Rows, Base> cholesky() const;
    _Matrix<Rows, Rows, Base> choleskyInverse() const;
    void svd(_Matrix<Rows, Cols, Base>& u, _Vector<Cols, Base>& s, _Matrix<Cols, Cols, Base>& v) const;

    Base det() const;

    void swapRows(int r1, int r2);
    void sumRow(int dest, Base destFactor, int src, Base srcFactor);
    void multRow(int dest, Base destFactor);
    void fill(Base scalar);
    
    _Array2DAllocator<Rows, Cols, Base> _allocator;

  };

template <int R, int C, typename Base>
  std::ostream& operator << (std::ostream& os, const _Matrix<R, C, Base>& m);

/** calculate scalar * Matrix, which equals to Matrix * scalar. */
template <int R, int C, typename Base>
_Matrix<R, C, Base> operator* (Base x, const _Matrix<R, C, Base>& m);

template <int M, int N, typename Base>
  void st2dyn(_Matrix<0,0,Base>& dest, const _Matrix<M,N,Base> src){
  st2dyn(dest._allocator, src._allocator);
}


template <int M, int N, typename Base>
  void dyn2st(_Matrix<M,N,Base>& dest, const _Matrix<0,0,Base> src){
  dyn2st(dest._allocator, src._allocator);
}

typedef _Matrix<3, 3, float>  Matrix3f;
typedef _Matrix<2, 2, float>  Matrix2f;
typedef _Matrix<6, 6, float>  Matrix6f;
typedef _Matrix<4, 4, float>  Matrix4f;
typedef _Matrix<3, 3, double> Matrix3;
typedef _Matrix<2, 2, double> Matrix2;
typedef _Matrix<6, 6, double> Matrix6;
typedef _Matrix<4, 4, double> Matrix4;

typedef _Matrix<0, 0, double> MatrixX;
typedef _Matrix<0, 0, float> MatrixXf;

//@}

#include "matrix_n.hpp"


#endif
