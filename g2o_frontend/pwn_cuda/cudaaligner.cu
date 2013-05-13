#include <sys/time.h>
#include <cstdio>
#include <iostream>

#include "cudasla.cuh"
#include "cudautils.cuh"
#include "cudaaligner.cuh"

namespace CudaAligner {
  using namespace std;


  void matPrint(const float* m, int r, int c, const char* msg){
    if (msg){
      printf("%s r:%d c:%d\n", msg, r, c);
    }
    for (int i=0; i<r; i++) {
      for (int j=0; j<c; j++) {
	printf("%f ",m[j*r+i]);
      }
      printf("\n");
    }
  }

  //#define __device__ inline
  //#define __host__ inline

  const char* AlignerStatus::strOperation[]  = {
    "Ok", 
    "Allocation", 
    "Deallocation", 
    "CopyToDevice", 
    "CopyFromDevice", 
    "KernelLaunch", 
    "InitComputation"
  };

  const char* AlignerStatus::strObject[] = {
    "NoObject", 
    "Reference", 
    "Current", 
    "Context", 
    "DepthBuffer", 
    "AccumulationBuffer",
    "ReductionBuffer",
  };

  const char* AlignerStatus::strObjectDetail[] =  {
    "NoDetail", 
    "Points", 
    "Normals", 
    "Curvatures", 
    "OmegaPs", 
    "OmegaNs", 
    "Indices"
  };

  void AlignerStatus::toString(char* s) {
    sprintf(s, "Operation: %s, Object: %s, ObjectDetail: %s", 
	    strOperation[_operation],
	    strObject[_object],
	    strObjectDetail[_detail]);
  }


#define allocateVar(vtype, name, size, errorcode)	\
  vtype * name;						\
  {							\
    err = cudaMalloc(& name, (size) * sizeof(vtype) );	\
    if ( err != cudaSuccess)				\
      return errorcode;					\
  }

#define freeVar(name, errorcode)		\
  {						\
    err = cudaFree(name);			\
    if ( err != cudaSuccess)			\
      return errorcode;				\
  }



  __host__ AlignerStatus AlignerContext::init(int maxReferencePoints_, int maxCurrentPoints_, int rows_, int cols_) {
  
    _inlierThreshold =  9e3;
    _distanceThreshold = 3.0f;
    _distanceThreshold *= _distanceThreshold;
    _normalThreshold = cos(M_PI/6);
    _flatCurvatureThreshold = 0.02f;
    _maxCurvatureRatio = 1.3f;
    _minCurvatureRatio = 1./_maxCurvatureRatio;
    _maxDepth = 100000;

    _numReferencePoints = 0;
    _numCurrentPoints = 0;
    _maxReferencePoints = maxReferencePoints_;
    _maxCurrentPoints = maxCurrentPoints_;
    _rows = rows_;
    _cols = cols_;
    _error = 0;
    _inliers =0;

    // allocate the things
    cudaError_t err;
    allocateVar(float, d_referencePointsPtr, _maxReferencePoints*4, AlignerStatus(Allocation,Reference,Points));
    allocateVar(float, d_referenceNormalsPtr, _maxReferencePoints*4, AlignerStatus(Allocation,Reference,Normals));
    allocateVar(float, d_referenceCurvaturesPtr, _maxReferencePoints, AlignerStatus(Allocation,Reference,Curvatures));
    allocateVar(float, d_currentPointsPtr, _maxCurrentPoints*4, AlignerStatus(Allocation,Current,Points));
    allocateVar(float, d_currentNormalsPtr, _maxCurrentPoints*4, AlignerStatus(Allocation,Current,Normals));
    allocateVar(float, d_currentCurvaturesPtr, _maxCurrentPoints, AlignerStatus(Allocation,Current,Curvatures));
    allocateVar(float, d_currentOmegaPsPtr, _maxCurrentPoints*16, AlignerStatus(Allocation,Current,OmegaPs));
    allocateVar(float, d_currentOmegaNsPtr, _maxCurrentPoints*16, AlignerStatus(Allocation,Current,OmegaNs));
    allocateVar(int,   d_referenceIndicesPtr, _rows*_cols, AlignerStatus(Allocation,Reference,Indices));
    allocateVar(int,   d_currentIndicesPtr, _rows*_cols,  AlignerStatus(Allocation,Current,Indices));
    allocateVar(int,   d_depthBufferPtr, _rows*_cols, AlignerStatus(Allocation,DepthBuffer));

  
    // construct the temp buffers
    const int bufferElement = 16*3+8;
    const int bsize = bufferElement*_rows*_cols;
    _accumulationBuffer = new float [bsize];
    allocateVar(float, d_accumulationBuffer, bsize*sizeof(float), AlignerStatus(Allocation,AccumulationBuffer));
    allocateVar(float, d_reductionBuffer,    bsize*sizeof(float), AlignerStatus(Allocation,AccumulationBuffer));

    err = cudaMalloc(&_cudaDeviceContext,sizeof(AlignerContext));
    if (err!=cudaSuccess)
      return AlignerStatus(Allocation,Context);

  
    _cudaHostContext = new AlignerContext;
    *_cudaHostContext = *this;
    _cudaHostContext->_referencePoints.map(4,_maxReferencePoints, d_referencePointsPtr);
    _cudaHostContext->_referenceNormals.map(4,_maxReferencePoints, d_referenceNormalsPtr);
    _cudaHostContext->_referenceCurvatures = d_referenceCurvaturesPtr;
    _cudaHostContext->_currentPoints.map(4,_maxCurrentPoints, d_currentPointsPtr);
    _cudaHostContext->_currentNormals.map(4,_maxCurrentPoints, d_currentNormalsPtr);
    _cudaHostContext->_currentCurvatures = d_currentCurvaturesPtr;
    _cudaHostContext->_currentOmegaPs.map(16,_maxCurrentPoints, d_currentOmegaPsPtr);
    _cudaHostContext->_currentOmegaNs.map(16,_maxCurrentPoints, d_currentOmegaNsPtr);
    _cudaHostContext->_currentIndices.map(_rows, _cols, d_currentIndicesPtr);


    _cudaHostContext->_referenceIndices.map(_rows, _cols, d_referenceIndicesPtr);
    _cudaHostContext->_currentIndices.map(_rows, _cols, d_currentIndicesPtr);
    _cudaHostContext->_depthBuffer.map(_rows, _cols, d_depthBufferPtr);
    _cudaHostContext->_accumulationBuffer=d_accumulationBuffer;
    _cudaHostContext->_reductionBuffer=d_reductionBuffer;

    err = cudaMemcpy(_cudaDeviceContext,_cudaHostContext, sizeof(AlignerContext), cudaMemcpyHostToDevice);
    if (err!=cudaSuccess)
      return AlignerStatus(CopyToDevice,Context);
    return AlignerStatus();
  }



  __host__ AlignerStatus AlignerContext::free(){
    cudaError_t err;
    freeVar(_cudaHostContext->_referencePoints.values(), AlignerStatus(Deallocation,Reference,Points));
    freeVar(_cudaHostContext->_referenceNormals.values(), AlignerStatus(Deallocation,Reference,Normals));
    freeVar(_cudaHostContext->_referenceCurvatures, AlignerStatus(Deallocation,Reference,Curvatures));
    freeVar(_cudaHostContext->_currentPoints.values(), AlignerStatus(Deallocation,Current,Points));
    freeVar(_cudaHostContext->_currentNormals.values(), AlignerStatus(Deallocation,Current,Normals));
    freeVar(_cudaHostContext->_currentCurvatures, AlignerStatus(Deallocation,Current,Curvatures));
    freeVar(_cudaHostContext->_currentOmegaPs.values(), AlignerStatus(Deallocation,Current,OmegaPs));
    freeVar(_cudaHostContext->_currentOmegaNs.values(), AlignerStatus(Deallocation,Current,OmegaNs));
    freeVar(_cudaHostContext->_referenceIndices.values(), AlignerStatus(Deallocation,Reference,Indices));
    freeVar(_cudaHostContext->_currentIndices.values(), AlignerStatus(Deallocation,Current,Indices));
    freeVar(_cudaHostContext->_depthBuffer.values(), AlignerStatus(Deallocation,DepthBuffer));
    freeVar(_cudaHostContext->_accumulationBuffer, AlignerStatus(Deallocation,AccumulationBuffer));
    freeVar(_cudaHostContext->_reductionBuffer, AlignerStatus(Deallocation,ReductionBuffer));
    delete [] _accumulationBuffer;
    freeVar(_cudaDeviceContext, AlignerStatus(Deallocation,Context));
    delete _cudaHostContext;
    _cudaHostContext=0;
    _cudaDeviceContext =0;
    return AlignerStatus();
  }



  // initializes the computation by passing all the values that will not change during the iterations
  __host__ AlignerStatus AlignerContext::initComputation(const float* cameraMatrix,
							 float* referencePointsPtr, 
							 float* referenceNormalsPtr, 
							 float* referenceCurvaturesPtr, 
							 int numReferencePoints_, 
							 float* currentPointsPtr, 
							 float* currentNormalsPtr, 
							 float* currentCurvaturesPtr, 
							 float* currentOmegaPsPtr, 
							 float* currentOmegaNsPtr, 
							 int numCurrentPoints_){
    //cerr << "INIT_CUDA_ALIGNER" << endl;
    if (numReferencePoints_>_maxReferencePoints)
      return AlignerStatus(InitComputation,Reference);
    if (numCurrentPoints_>_maxCurrentPoints)
      return AlignerStatus(InitComputation,Current);
    
    _numReferencePoints = numReferencePoints_;
    _numCurrentPoints = numCurrentPoints_;
    _cudaHostContext->_numReferencePoints = _numReferencePoints;
    _cudaHostContext->_numCurrentPoints = _numCurrentPoints;

    // adjust the pointers on the host
    _referencePoints.map(4,_numReferencePoints,referencePointsPtr);
    _referenceNormals.map(4,_numReferencePoints,referenceNormalsPtr);
    _referenceCurvatures=referenceCurvaturesPtr;
    _currentPoints.map(4,_numCurrentPoints,currentPointsPtr);
    _currentNormals.map(4,_numCurrentPoints,currentNormalsPtr);
    _currentCurvatures=currentCurvaturesPtr;
    _currentOmegaPs.map(16,_numCurrentPoints, currentOmegaPsPtr);
    _currentOmegaNs.map(16,_numCurrentPoints, currentOmegaNsPtr);

    // adjust the parameters on the host copy of the device, without changing the buffers
    _cudaHostContext->_referencePoints.map(4,_numReferencePoints);
    _cudaHostContext->_referenceNormals.map(4,_numReferencePoints);
    _cudaHostContext->_currentPoints.map(4,_numCurrentPoints);
    _cudaHostContext->_currentNormals.map(4,_numCurrentPoints);
    _cudaHostContext->_currentOmegaPs.map(16,_numCurrentPoints);
    _cudaHostContext->_currentOmegaNs.map(16,_numCurrentPoints);

    vecFill<16>(_cameraMatrix,0.0f);
    for (int c=0; c<3; c++)
      for (int r=0; r<3; r++){
	_cameraMatrix[4*c+r]=cameraMatrix[3*c+r];
      }
    vecCopy<16>(_cudaHostContext->_cameraMatrix, _cameraMatrix);
    
    // do the copy of the buffers
    cudaError_t err ;
    err = cudaMemcpy(_cudaHostContext->_referencePoints.values(), 
		     _referencePoints.values(), 4*sizeof(float)*_numReferencePoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Reference,Points);

    err = cudaMemcpy(_cudaHostContext->_referenceNormals.values(), 
		     _referenceNormals.values(), 4*sizeof(float)*_numReferencePoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Reference,Normals);
    err = cudaMemcpy(_cudaHostContext->_referenceCurvatures, 
		     _referenceCurvatures, sizeof(float)*_numReferencePoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Reference,Curvatures);

    err = cudaMemcpy(_cudaHostContext->_currentPoints.values(), 
		     _currentPoints.values(), 4*sizeof(float)*_numCurrentPoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Current,Points);

    err = cudaMemcpy(_cudaHostContext->_currentNormals.values(), 
		     _currentNormals.values(), 4*sizeof(float)*_numCurrentPoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Current,Normals);

    err = cudaMemcpy(_cudaHostContext->_currentCurvatures, 
		     _currentCurvatures, sizeof(float)*_numCurrentPoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Current,Curvatures);

    err = cudaMemcpy(_cudaHostContext->_currentOmegaPs.values(), 
		     _currentOmegaPs.values(), 16*sizeof(float)*_numCurrentPoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Current,OmegaPs);

    err = cudaMemcpy(_cudaHostContext->_currentOmegaNs.values(), 
		     _currentOmegaNs.values(), 16*sizeof(float)*_numCurrentPoints, cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      return AlignerStatus(CopyToDevice,Current,OmegaNs);
  
    // now do the copy of the high level structure (which includes all the internal variables, including
    // the parameters
    err = cudaMemcpy(_cudaDeviceContext,_cudaHostContext, sizeof(AlignerContext), cudaMemcpyHostToDevice);
    if (err!=cudaSuccess)
      return AlignerStatus(CopyToDevice,Context);

    struct timeval tvStart, tvEnd;
    int numElements = _rows*_cols;
    int threadsPerBlock = 256;
    int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
    //printf("clearBuffer: CUDA kernel launch on %d elements with %d blocks of %d threads\n", numElements, blocksPerGrid, threadsPerBlock);
    gettimeofday(&tvStart,0);
    //cerr << "clearDepth" << endl;
    fillBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_depthBuffer.values(),_rows*_cols, 
						  _maxDepth+1);
    err = cudaGetLastError();
    if (err != cudaSuccess)
      {
	return AlignerStatus(KernelLaunch,DepthBuffer);
      }

    //cerr << "clearIndices" << endl;
    fillBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_currentIndices.values(),_rows*_cols, 
						  -1);
    err = cudaGetLastError();
    if (err != cudaSuccess)
      {
	return AlignerStatus(KernelLaunch,DepthBuffer);
      }
						    
    cudaDeviceSynchronize();
      
    numElements = _numCurrentPoints;
    threadsPerBlock = 256;
    blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
    //printf("Buffer: CUDA kernel launch on %d elements with %d blocks of %d threads\n", numElements, blocksPerGrid, threadsPerBlock);

    //cerr << "computeDepth" << endl;
    fillBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_currentIndices.values(),_rows*_cols, 
						  -1);
    computeDepthBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_depthBuffer.values(),
							  _rows, _cols, _cudaDeviceContext->_cameraMatrix,
							  _cudaHostContext->_currentPoints.values(),
							  _cudaHostContext->_numCurrentPoints,
							  0, _cudaHostContext->_maxDepth);
    err = cudaGetLastError();
    if (err != cudaSuccess)
      {
	return AlignerStatus(KernelLaunch,DepthBuffer);
      }
    cudaDeviceSynchronize();
    //cerr << "computeIndices" << endl;
    computeIndexBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_currentIndices.values(),
							  _cudaHostContext->_depthBuffer.values(),
							  _rows, _cols, _cudaDeviceContext->_cameraMatrix,
							  _cudaHostContext->_currentPoints.values(),
							  _cudaHostContext->_numCurrentPoints);
    err = cudaGetLastError();
    if (err != cudaSuccess)
      {
	return AlignerStatus(KernelLaunch,DepthBuffer);
      }

    err = cudaGetLastError();
    if (err != cudaSuccess)
      {
	return AlignerStatus(KernelLaunch,DepthBuffer);
      }
    cudaDeviceSynchronize();
    gettimeofday(&tvEnd,0);
    //double tStart = tvStart.tv_sec*1000+tvStart.tv_usec*0.001;
    //double tEnd = tvEnd.tv_sec*1000+tvEnd.tv_usec*0.001;
    //printf("\t zbuffer time: %lf",tEnd-tStart);

    return  AlignerStatus();
  }

  __global__ void Aligner_coolIterationKernel(AlignerContext* context){
    extern __shared__ float sdata[];
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    const int dim = 56;
    int sdataOffset = threadIdx.x*dim;
    float x[dim];
    
    //int retval = 0;
    if (i < context->_rows*context->_cols) {
      vecFill<56>(x, 0.0f);
      float error;
      float* Htt = x;
      float* Htr = x + 16;
      float* Hrr = x + 32;
      float* bt  = x + 48;
      float* br  = x + 52;

      int referenceIndex=*(context->_referenceIndices.values()+i);
      int currentIndex=*(context->_currentIndices.values()+i);
      if (referenceIndex >=0  && currentIndex >=0){
	/*retval = */
	context->processCorrespondence(&error, Htt, Hrr, Htr,
				       bt, br,
				       referenceIndex, 
				       currentIndex);
      }
    }
    vecCopy<dim>(sdata+sdataOffset,x);
    __syncthreads();
    float* reductionBuffer = context->_reductionBuffer;

    __syncthreads();
    if (threadIdx.x<dim){
      float xi = 0;
      float  *sptr = sdata+threadIdx.x;
      for (int j=0; j<blockDim.x; j++){
    	xi += *sptr;
    	sptr += dim;
      }
      *(reductionBuffer + (blockIdx.x*dim+threadIdx.x)) = xi;
    }
    __syncthreads();
  };





  __host__ AlignerStatus AlignerContext::simpleIteration(float* transform){

    struct timeval tvStart, tvEnd;
    cudaError_t err ;
    gettimeofday(&tvStart,0);

    // resize the local matrices (kinda useless)
    _referenceIndices.map(_rows,_cols);
    _currentIndices.map(_rows,_cols);

    // copy the reference indices and the current indices in the context buffers
    /*
      _referenceIndices.map(_rows,_cols,referenceIndices);
      err = cudaMemcpy(_cudaHostContext->_referenceIndices.values(), _referenceIndices.values(), _rows*_cols*sizeof(int), cudaMemcpyHostToDevice);
      if (err!=cudaSuccess)
      return AlignerStatus(CopyToDevice,Reference, Indices);
    */

    /*
      _currentIndices.map(_rows,_cols,currentIndices);
      err = cudaMemcpy(_cudaHostContext->_currentIndices.values(), _currentIndices.values(), _rows*_cols*sizeof(int), cudaMemcpyHostToDevice);
      if (err!=cudaSuccess)
      return AlignerStatus(CopyToDevice,Current, Indices);
    */

    vecCopy<16>(_transform, transform);
    vecCopy<16>(_cudaHostContext->_transform, transform);
    
    vecFill<16>(_KT,0.0f);
    matMatMul<4,4,4>(_KT,_cameraMatrix,_transform);
    matVecMul<4,4>(_KT+12,_cameraMatrix,_transform+12);
    vecCopy<16>(_cudaHostContext->_KT, _KT);
    
    //matPrint(_cudaHostContext->_transform,4,4,"Transform");
    //matPrint(_cudaHostContext->_cameraMatrix,4,4,"_cameraMatrix");
    //matPrint(_cudaHostContext->_KT,4,4,"KT");

    _cudaHostContext->_checksum = 0;
    _cudaHostContext->_maxDepth = 10000;
    err = cudaMemcpy(_cudaDeviceContext,_cudaHostContext, sizeof(AlignerContext), cudaMemcpyHostToDevice);
    //matPrint(_cudaHostContext->_KT,4,4,"KT:");
    if (err!=cudaSuccess)
      return AlignerStatus(CopyToDevice,Context);

    if (1){
      int numElements = _rows*_cols;
      int threadsPerBlock = 256;
      int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
      //printf("clearBuffer: CUDA kernel launch on %d elements with %d blocks of %d threads\n", numElements, blocksPerGrid, threadsPerBlock);
      fillBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_depthBuffer.values(),_rows*_cols, 
						    _maxDepth+1);
      fillBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_referenceIndices.values(),_rows*_cols, 
       						    -1);
						    
      //cudaDeviceSynchronize();
      
      numElements = _numReferencePoints;
      threadsPerBlock = 256;
      blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
      //printf("Buffer: CUDA kernel launch on %d elements with %d blocks of %d threads\n", numElements, blocksPerGrid, threadsPerBlock);
      
      //computeAlignerDepthBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaDeviceContext);
      /*
	__device__ void computeDepthBuffer(int* buffer, int rows, int cols, 
	const float* KT, const float* points, int numPoints,
	int dmin, int dmax);
      */
      computeDepthBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_depthBuffer.values(),
							    _rows, _cols, _cudaDeviceContext->_KT,
							    _cudaHostContext->_referencePoints.values(),
							    _cudaHostContext->_numReferencePoints,
							    0, _cudaHostContext->_maxDepth);
      //cudaDeviceSynchronize();
      computeIndexBuffer<<<blocksPerGrid,threadsPerBlock>>>(_cudaHostContext->_referenceIndices.values(),
							    _cudaHostContext->_depthBuffer.values(),
							    _rows, _cols, _cudaDeviceContext->_KT,
							    _cudaHostContext->_referencePoints.values(),
							    _cudaHostContext->_numReferencePoints);
      //cudaDeviceSynchronize();

      
      /*cudaMemcpy(_referenceIndices.values(), _cudaHostContext->_referenceIndices.values(), _rows * _cols * sizeof(float), cudaMemcpyDeviceToHost);
       */

      err = cudaGetLastError();
      if (err != cudaSuccess)
	{
	  return AlignerStatus(KernelLaunch,DepthBuffer);
	}

    }
    

  
  
    int numElements = _rows * _cols;
    int threadsPerBlock = 64;
    int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
    int sharedMemorySize = threadsPerBlock * 56 * sizeof(float);
    //printf("CUDA kernel launch with %d blocks of %d threads\n", blocksPerGrid, threadsPerBlock);
    Aligner_coolIterationKernel<<<blocksPerGrid,threadsPerBlock, sharedMemorySize>>>(_cudaDeviceContext); 
    //Aligner_simpleIterationKernel<<<blocksPerGrid,threadsPerBlock>>>(_cudaDeviceContext); 
    err = cudaGetLastError();
    if (err != cudaSuccess)
      {
	return AlignerStatus(KernelLaunch);
      }
    //cudaDeviceSynchronize();
    numElements = blocksPerGrid;
    while(numElements>1){
      int threadsPerBlock = 32;
      int sharedMemorySize = threadsPerBlock * 56 * sizeof(float);
      blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
      //printf("CUDA kernel launch with %d blocks of %d threads on %d elements\n", blocksPerGrid, threadsPerBlock, numElements);
      vector_block_sum<56><<<blocksPerGrid, threadsPerBlock, sharedMemorySize>>>(_cudaHostContext->_reductionBuffer,
										 _cudaHostContext->_reductionBuffer,
										 numElements);
      if (err != cudaSuccess)
	{
	  return AlignerStatus(KernelLaunch);
	}
      numElements = blocksPerGrid;
    }
    //cudaDeviceSynchronize();
    cudaMemcpy(_cudaDeviceContext->_Hb, _cudaHostContext->_reductionBuffer, 56*sizeof(float), cudaMemcpyDeviceToDevice);
    
    err = cudaMemcpy(_cudaHostContext,_cudaDeviceContext, sizeof(AlignerContext), cudaMemcpyDeviceToHost);
    if (err!=cudaSuccess)
      return AlignerStatus(CopyFromDevice,Context);
    gettimeofday(&tvEnd,0);
    fflush(stdout);

    double tStart = tvStart.tv_sec*1000+tvStart.tv_usec*0.001;
    double tEnd = tvEnd.tv_sec*1000+tvEnd.tv_usec*0.001;
    _inliers = *(_cudaHostContext->_Hb+16*3+3);
    _error   = *(_cudaHostContext->_Hb+16*3+4+3);
    _time = tEnd-tStart;
    //cerr << "\ttime: " << tEnd-tStart;
    //cerr << " inliers: " << *(_cudaHostContext->_Hb+16*3+3);
    //cerr << " error  : " << *(_cudaHostContext->_Hb+16*3+4+3) << endl;

    return AlignerStatus();
  }


  __device__ int AlignerContext::processCorrespondence(float* error,
						       float* Htt,
						       float* Hrr,
						       float* Htr,
						       float* bt,
						       float* br,
						       int referenceIndex, int currentIndex){

    vecFill<16>(Htt, 0);
    vecFill<16>(Htr, 0);
    vecFill<16>(Hrr, 0);
    vecFill<4>(bt,0);
    vecFill<4>(br,0);
    const float* T=_transform;
    float referencePoint[4];
    {
      const float* referencePoint_  =  _referencePoints.columnAt(referenceIndex);
      matVecMul<4,4>(referencePoint,T,referencePoint_);
    }

    float referenceNormal[4];
    {
      const float* referenceNormal_ =  _referenceNormals.columnAt(referenceIndex);
      matVecMul<4,4>(referenceNormal,T,referenceNormal_);
    }

    float referenceCurvature      =  _referenceCurvatures[referenceIndex];
    referenceCurvature = (referenceCurvature<_flatCurvatureThreshold)?_flatCurvatureThreshold:referenceCurvature;


    const float* currentPoint     =  _currentPoints.columnAt(currentIndex);
    const float* currentNormal    =  _currentNormals.columnAt(currentIndex);
    float   currentCurvature      =  _currentCurvatures[currentIndex];
    currentCurvature = (currentCurvature<_flatCurvatureThreshold)?_flatCurvatureThreshold:currentCurvature; 

    const float* omegaP           =  _currentOmegaPs.columnAt(currentIndex);
    const float* omegaN           =  _currentOmegaNs.columnAt(currentIndex);
    float pointsDifference[4];



    float curvatureRatio=(referenceCurvature + 1e-5)/(currentCurvature + 1e-5);
    float normalsRatio = vecDot<4>(currentNormal,referenceNormal);
  
    vecCopy<4>(pointsDifference,referencePoint);
    vecSum<4>(pointsDifference,currentPoint,-1.0f);
    float pointsDistance=vecDot<4>(pointsDifference,pointsDifference);


    bool normalFail = (normalsRatio < _normalThreshold);
    bool distanceFail = (pointsDistance > _distanceThreshold);
    bool curvatureFail = ((curvatureRatio < _minCurvatureRatio) ||
			  (curvatureRatio > _maxCurvatureRatio));

    int increment = ! normalFail && ! distanceFail && ! curvatureFail;
    if (! increment)  
      return 0;

    float normalsDifference[4];
    vecCopy<4>(normalsDifference,referenceNormal);
    vecSum<4>(normalsDifference,currentNormal,-1.0f);
  

    //const Vector4f ep = omegaP*pointError;
    float ep[4];
    matVecMul<4,4>(ep, omegaP, pointsDifference);

    //const Vector4f en = omegaN*normalError;
    float en[4];
    matVecMul<4,4>(en, omegaN, normalsDifference);
  
    float localError = vecDot<4>(ep,pointsDifference) + vecDot<4>(en,normalsDifference);

    int chiOk = localError < _inlierThreshold;
    if (! chiOk)
      return 0;
  
    //Matrix4f Sp = skew(referencePoint);
    float Sp[16];
    matBuildSkew(Sp,referencePoint);

    //Matrix4f Sn = skew(referenceNormal);
    float Sn[16];
    matBuildSkew(Sn,referenceNormal);


    //Htt = omegaP;
    vecCopy<16>(Htt,omegaP);
  
    //Htr.noalias() = omegaP*Sp;
    matMatMul<4,4,4>(Htr,omegaP,Sp);

    //Hrr.noalias() = - (Sp*omegaP*Sp + Sn*omegaN*Sn);
    float temp[16], temp2[16];
    matMatMul<4,4,4>(Hrr,Sp,Htr);
    matMatMul<4,4,4>(temp,Sn,omegaN);
    matMatMul<4,4,4>(temp2,temp,Sn);
    vecSum<16>(Hrr,temp2,+1.0f);
    vecScale<16>(Hrr,-1.0f);
    //bt.noalias() = ep;

    vecCopy<4>(bt,ep);


    //br.noalias() = - (Sp*ep + Sn*en);
    matVecMul<4,4>(br,Sp,ep);
    matVecMul<4,4>(temp,Sn,en);
    vecSum<4>(br,temp,+1.0f);
    vecScale<4>(br,-1.0f);
    *error = localError;
    *(bt+3)=1;
    *(br+3)= localError;
    return 1;
  }

#undef allocateVar
#undef freeVar


  __host__ AlignerStatus createContext(AlignerContext** context, int maxReferencePoints, int maxCurrentPoints, int rows, int cols){
    *context = new AlignerContext;
    return (*context)->init(maxReferencePoints, maxCurrentPoints, rows, cols);
  }

  __host__ AlignerStatus destroyContext(AlignerContext* context){
    AlignerStatus s = context->free();
    delete context;
    return s;
  }

  __host__  AlignerStatus initComputation(AlignerContext* context,
					  const float* cameraMatrix,
					  float* referencePointsPtr, 
					  float* referenceNormalsPtr, 
					  float* referenceCurvaturesPtr, 
					  int numReferencePoints_, 
					  float* currentPointsPtr, 
					  float* currentNormalsPtr, 
					  float* currentCurvaturesPtr, 
					  float* currentOmegaPsPtr, 
					  float* currentOmegaNPtr, 
					  int numCurrentPoints_){

    return context->initComputation(cameraMatrix,
				    referencePointsPtr, 
				    referenceNormalsPtr, 
				    referenceCurvaturesPtr,
				    numReferencePoints_, 
				    currentPointsPtr, 
				    currentNormalsPtr, 
				    currentCurvaturesPtr, 
				    currentOmegaPsPtr, 
				    currentOmegaNPtr,
				    numCurrentPoints_);
  }

  __host__  AlignerStatus simpleIteration(AlignerContext* context,
					  float* transform, int* inliers, float* error ){
    AlignerStatus status=context->simpleIteration(transform);
    *error = context->error();
    *inliers = context->inliers();
    return status;
  }

  int AlignerContext::getHb(float* Htt, float* Htr, float* Hrr, float*bt, float* br){
    vecCopy<16>(Htt, _cudaHostContext->_Hb);
    vecCopy<16>(Htr, _cudaHostContext->_Hb+16);
    vecCopy<16>(Hrr, _cudaHostContext->_Hb+32);
    vecCopy<4>(bt,   _cudaHostContext->_Hb+48);
    vecCopy<4>(br,   _cudaHostContext->_Hb+52);
    
    return _rows*_cols;
  }


  int getHb(AlignerContext* context, float* Htt, float* Hrt, float* Hrr, float*bt, float* br){
    return context->getHb(Htt, Hrt, Hrr, bt, br);
  }

} // end namespace

