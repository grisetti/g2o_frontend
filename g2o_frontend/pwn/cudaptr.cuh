#ifndef __CUDAPTR_CU__
#define __CUDAPTR_CU__
#include <assert.h>
#include <cuda_runtime.h>

template <class T>
struct CudaPointer{
  __host__  inline T* hptr() {return _hostPtr;}

  __host__  inline const T* hptr() const {return _hostPtr;}

  __device__  inline T* dptr() {return _devicePtr;}

  __device__  inline const T* dptr() const {return _devicePtr;}

  __host__ inline void init(){
    _size = 0;
    _hostPtr = 0;
    _devicePtr = 0;
  }

  __host__ inline cudaError_t map(T* hostPtr_, int size_, int _maxSize){
    cudaError_t err;
    if (_hostPtr == hostPtr_ && _size == size_ && _devicePtr)
      return cudaSuccess;

    if ( (_hostPtr != hostPtr_  || _size > _maxSize ) &&
	 _devicePtr) {
      err = cudaFree(_devicePtr);
      if (err !=  cudaSuccess)
	return err;
      _hostPtr = hostPtr_;
      _size = size_;
      _maxSize = size_;
    }
    
    if (!hostPtr_){
      _size = 0;
      _hostPtr = 0;
      return cudaSuccess;
    }
    _size = size_;
    _hostPtr=hostPtr_;
    err = cudaMalloc((void**)&_devicePtr, _size*sizeof(T));
    if (err != cudaSuccess)
      return err;
    printf("alloc ok  host: %lx device: %lx \n", _hostPtr , _devicePtr);
    return cudaSuccess;
  }
  
  __host__ cudaError_t copyToDevice(){
    printf("copy h->d ok  host: %lx device: %lx \n", _hostPtr , _devicePtr);
    cudaError_t err = cudaMemcpy(_devicePtr, _hostPtr, _size*sizeof(T), cudaMemcpyHostToDevice);
    return err;
  }
  
  __host__ cudaError_t copyFromDevice(){
    printf("copy d->h ok  host: %lx device: %lx \n", _hostPtr , _devicePtr);
    cudaError_t err = cudaMemcpy(_hostPtr, _devicePtr, _size*sizeof(T), cudaMemcpyDeviceToHost);
    return err;
  }

  __host__ __device__ size_t size() const {return _size;} 

  __host__ __device__ size_t maxSize() const {return _maxSize;} 
protected:

  T* _hostPtr, *_devicePtr;
  size_t _size;
  size_t _maxSize;
};

typedef CudaPointer<int> CudaIntPtr;
typedef CudaPointer<float> CudaFloatPtr;

#endif
