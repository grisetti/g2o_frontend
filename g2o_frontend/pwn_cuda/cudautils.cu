namespace CudaAligner{

  template <class T>
  __global__ void fillBuffer(T* buffer, int numElems, T value){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i<numElems){
      buffer[i] = value;
    }
  }


  __global__ void computeDepthBuffer(int* buffer, int rows, int cols, 
				     const float* KT, const float* points, int numPoints,
				     int dmin, int dmax){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    float ip[4];
    if (i<numPoints){
      const float* wp=points+4*i;
      CudaAligner::matVecMul<4,4>(ip,KT,wp);
      float iw= 1./ip[2];
      int d = ip[2] * 1000.0f;
      int x = (int)(ip[0]*iw);
      int y = (int)(ip[1]*iw);
      int offset = y*rows+x;
      if (d > dmin && d < dmax &&
	  x >= 0 && x<rows && 
	  y>=0 && y<cols){
	atomicMin(buffer+offset,d);
      }
    }
  }


  __global__ void computeIndexBuffer(int* ibuffer, const int* zbuffer, int rows, int cols, 
				     const float* KT, const float* points, int numPoints){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    float ip[4];
    if (i<numPoints){
      const float* wp=points+4*i;
      CudaAligner::matVecMul<4,4>(ip,KT,wp);
      float iw= 1./ip[2];
      int d = ip[2] * 1000.0f;
      int x = (int)(ip[0]*iw);
      int y = (int)(ip[1]*iw);
      int offset = y*rows+x;
      if ( x >= 0 && x<rows && 
	   y>=0 && y<cols && d == zbuffer[offset]){
	ibuffer[offset]=i;
      }
    }
  }

}
