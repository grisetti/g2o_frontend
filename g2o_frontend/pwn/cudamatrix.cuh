#ifndef _CUDA_MATRIX_CUH_
#define _CUDA_MATRIX_CUH_
#include "cudaptr.cuh"

template <typename T, int RowsAtCompileTime>
struct CudaMatrix{
  __host__ inline void init() {
    _rows = RowsAtCompileTime == -1 ? 0 : RowsAtCompileTime;
    _cols = 0;
    _values.init();
  }
  
  __host__ cudaError_t map(int r, int c, T* x){
    if (RowsAtCompileTime!=-1 && RowsAtCompileTime != r){
      assert (0 && "this is a fixed size column matrix, you cant give it arbitrary column size");
    }
    _rows = r;
    _cols = c;
    return _values.map(x,_rows*_cols);
  }
  
  __device__ inline const T* rowAt(int i) const {
    if (RowsAtCompileTime == -1){
      return _values.dptr()+(RowsAtCompileTime*i);
    } else {
      return _values.dptr()+(_rows*i);
    }
  }

  __device__  inline T* rowAt(int i) {
    if (RowsAtCompileTime == -1){
      return _values.dptr()+(RowsAtCompileTime*i);
    } else {
      return _values.dptr()+(_rows*i);
    }
  }
  
  __device__  inline T elementAt(int r, int c) const {return *(rowAt(c)+r);}

  __device__  inline void setElementAt(int r, int c, const T& v) const {*(rowAt(c)+r) = v;}

  __host__ __device__  inline int rows() const {return _rows;}

  __host__ __device__  inline int cols() const {return _cols;}

  __host__ cudaError_t copyToDevice() {
    
  }

  __host__
protected:

  int _cols;
  int _rows;
  CudaPointer<T> _values;
};


typedef CudaMatrix<float,16> MatrixFloat16x;
typedef CudaMatrix<float,4> MatrixFloat4x;
typedef CudaMatrix<int,-1>  MatrixIntXX;

#endif
