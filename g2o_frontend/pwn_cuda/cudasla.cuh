#ifndef _CUDASLA_CUH_
#define _CUDASLA_CUH_

namespace CudaAligner {

  __host__ __device__ void vecFill(float* v, float x, int n);

  __host__ __device__ void vecScale(float* v, float s, int n);

  __host__ __device__ void vecCopy(float* dest, const float* src, int n);

  __host__ __device__ void vecSum(float*dest, const float* src, float scale, int n);

  __host__ __device__ float vecDot(const float* v1, float* v2, int n);

  __host__ __device__ void matVecMul(float* dest, const float* A, const float*b, int rows, int cols);

  __host__ __device__ void matMatMul(float* dest, const float* A, const float*B, int ra, int ca, int cb);

  __host__ __device__ void matTranspose(float* dest, const float* src, int rows, int cols);


  template <int n>
  __host__ __device__ void vecFill(float* v, float x);

  template <int n>
  __host__ __device__ void vecScale(float* v, float s);
  
  template <int n>
  __host__ __device__ void vecCopy(float* dest, const float* src);
  
  template <int n>
  __host__ __device__ void vecSum(float*dest, const float* src, float scale);
  
  template <int n>
  __host__ __device__ float vecDot(const float* v1, const float* v2);
  
  template <int rows, int cols>
  __host__ __device__ void matVecMul(float* dest, const float* A, const float*b);
  
  template <int ra, int ca, int cb>
  __host__ __device__ void matMatMul(float* dest, const float* A, const float*B);
  
  template <int rows, int cols>
  __host__ __device__ void matVecMul3(float* dest, const float* A, const float*b);

  template <int ra, int ca, int cb>
  __host__ __device__ void matMatMul3(float* dest, const float* A, const float*B);

  template <int rows, int cols>
  __host__ __device__ void matTranspose(float* dest, const float* src);

  __host__ __device__ void matBuildSkew(float* m, const float* v);
  
  __host__ __device__ void transformInverse(float* d, const float* s);

  __host__ __device__ void _v2t(float* m, const float* v);

  __host__ __device__ void _t2v(float* v, const float* m);


  template <int dim>
  __global__ void vector_block_sum(const float *input,
				   float *per_block_results,
				   const size_t n);
} // end namespace

#include "cudasla.cu"

#endif
