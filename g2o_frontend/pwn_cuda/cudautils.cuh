#ifndef _CUDAUTILS_CUH_
#define _CUDAUTILS_CUH_
#include "cudasla.cuh"

namespace CudaAligner{
template <class T>
__global__ void fillBuffer(T* buffer, int numElems, T value);

__global__ void computeDepthBuffer(int* buffer, int rows, int cols, 
				   const float* KT, const float* points, int numPoints,
				   int dmin, int dmax);

__global__ void computeIndexBuffer(int* ibuffer, const int* zbuffer, int rows, int cols, 
				   const float* KT, const float* points, int numPoints);


}

#include "cudautils.cu"

#endif 
