#ifndef _CUDAMATRIX_CUH_
#define _CUDAMATRIX_CUH_

template <class T, int RowsAtCompileTime>
struct Matrix {
  inline __host__ void map(int r, int c, T* v=0){
    assert(r == RowsAtCompileTime);
    _cols = c;
    if (v)
      _values = v;
  }
  
  __host__ __device__  inline int pointAt(int r, T c) const {
    return *(_values+c*RowsAtCompileTime+r);
  }

  __host__ __device__ inline void setPointAt(int r, int c, T p){
    return *(_values+c*RowsAtCompileTime+r) = p;
  }

  __host__ __device__ inline const T* columnAt(int c) const{
    return _values+(c*RowsAtCompileTime);
  }

  __host__ __device__ inline T* columnAt(int c) {
    return _values+ (c*RowsAtCompileTime);
  }
  __host__ __device__ inline int rows() const {return RowsAtCompileTime;}
  __host__ __device__ inline int cols() const {return _cols;}
  __host__ __device__ inline const T* values() const {return _values;}
  __host__ __device__ inline T* values() { return _values;}
protected:
  int _cols;
  T* _values;
};


template <class T>
struct Matrix<T, -1> {
  inline __host__ void map(int r, int c, T* v=0){
    _rows = r;
    _cols = c;
    if (v)
      _values = v;
  }
  
  __host__ __device__  inline int pointAt(int r, T c) const {
    return *(_values+c*_rows+r);
  }

  __host__ __device__ inline void setPointAt(int r, int c, T p){
    return *(_values+c*_rows+r) = p;
  }

  __host__ __device__ inline const T* columnAt(int c) const{
    return _values+(c*_rows);
  }

  __host__ __device__ inline T* columnAt(int c) {
    return _values+(c*_rows);
  }
  __host__ __device__ inline int rows() const {return _rows;}
  __host__ __device__ inline int cols() const {return _cols;}
  __host__ __device__ inline const T* values() const {return _values;}
  __host__ __device__ inline T* values() { return _values;}
protected:
  int _cols;
  int _rows;
  T* _values;
};



typedef Matrix<int,-1> IntMatrix;
typedef Matrix<float,4> FloatMatrix4N;
typedef Matrix<float,16> FloatMatrix16N;

#endif
