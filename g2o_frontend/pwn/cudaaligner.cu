#include "cudaaligner.cuh"
#include <cstdio>
using namespace std;

#include "cudaaligner.cuh"
#include "cudasla.cuh"
#include <sys/time.h>

namespace CudaAligner {

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
  "AccumulationBuffer"
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
#define allocateVar(vtype, name, size, errorcode) \
  vtype * name; \
  { \
    err = cudaMalloc(& name, (size) * sizeof(vtype) ); \
    if ( err != cudaSuccess) \
      return errorcode; \
  }

#define freeVar(name, errorcode) \
  { \
    err = cudaFree(name); \
    if ( err != cudaSuccess) \
      return errorcode; \
  }



__host__ AlignerStatus AlignerContext::init(int maxReferencePoints_, int maxCurrentPoints_, int rows_, int cols_) {
  
  _inlierThreshold =  9e3;
  _distanceThreshold = 3.0f;
  _distanceThreshold *= _distanceThreshold;
  _normalThreshold = cos(M_PI/6);
  _flatCurvatureThreshold = 0.02f;
  _maxCurvatureRatio = 1.3f;
  _minCurvatureRatio = 1./_maxCurvatureRatio;

  _numReferencePoints = 0;
  _numCurrentPoints = 0;
  _maxReferencePoints = maxReferencePoints_;
  _maxCurrentPoints = maxCurrentPoints_;
  _rows = rows_;
  _cols = cols_;
  

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
  delete [] _accumulationBuffer;
  freeVar(_cudaDeviceContext, AlignerStatus(Deallocation,Context));
  delete _cudaHostContext;
  _cudaHostContext=0;
  _cudaDeviceContext =0;
  return AlignerStatus();
}



// initializes the computation by passing all the values that will not change during the iterations
__host__ AlignerStatus AlignerContext::initComputation(
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
  if (numReferencePoints_>_maxReferencePoints)
    return AlignerStatus(InitComputation,Reference);
  if (numCurrentPoints_>_maxCurrentPoints)
    return AlignerStatus(InitComputation,Current);

  _numReferencePoints = numReferencePoints_;
  _numCurrentPoints = numCurrentPoints_;

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
  return  AlignerStatus();
}
  /*
  __global__ void computeDepthBuffer(int* depthBuffer, float* cameraMatrix float* points, int numPoints, int maxDepth, int rows, int cols){
  }

  __global__ void computeIndexBuffer(int* indexBuffer, int* depthBuffer, float* cameraMatrix float* points, int numPoints, int rows, int cols){
  }
  */

  __global__ void Aligner_clearDepthBuffer(AlignerContext* context){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i<context->_rows * context->_cols){
      context->_depthBuffer.values()[i]=context->_maxDepth + 1;
    }
    __syncthreads();
  }

  __global__ void Aligner_computeReferenceDepthBuffer(AlignerContext* context, float* KT){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    float point[4];
    if (i<context->_numReferencePoints){
      float* referencePoint=context->_referencePoints.columnAt(i);
      matVecMul<4,4>(point,KT,referencePoint);
      float iw= 1./point[2];
      int d = 1000 * point[2];
      int x = (int)(point[0]*iw);
      int y = (int)(point[1]*iw);
      int offset = y*context->_rows+x;
      if ( d > 0 && d < context-> _maxDepth && offset >= 0  && offset <= context->_rows * context->_cols){
	  atomicMin(context->_depthBuffer.values()+offset, d);
      }
	__syncthreads();
    }
  }

  __global__ void Aligner_computeReferenceIndexBuffer(AlignerContext* context, float* KT){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    float point[4];
    if (i<context->_numReferencePoints){
      float* referencePoint=context->_referencePoints.columnAt(i);
      matVecMul<4,4>(point,KT,referencePoint);
      float iw= 1./point[2];
      int d = 1000 * point[2];
      int x = (int)(point[0]*iw);
      int y = (int)(point[1]*iw);
      int offset = y*context->_rows+x;
      if ( d > 0 && d < context-> _maxDepth && offset >= 0  && offset <= context->_rows * context->_cols){
	  int otherDptr = *(context->_depthBuffer.values()+offset);
	  if (abs(otherDptr-d)<1)
	    context->_referencePoints.values()[offset]=i;
      }
	__syncthreads();
      }
  }



  __global__ void Aligner_simpleIterationKernel(AlignerContext* context){
  int i = blockDim.x * blockIdx.x + threadIdx.x;
 
  if (i < context->_rows*context->_cols) {

    int retval = 0;
    float error;
    float Htt[16];
    float Hrr[16];
    float Htr[16];
    float bt[4];
    float br[4];

    int referenceIndex=*(context->_referenceIndices.values()+i);
    int currentIndex=*(context->_currentIndices.values()+i);
    atomicAdd(&context->_checksum, referenceIndex - currentIndex);
    
    vecFill<16>(Htt,0);
    vecFill<16>(Htr,0);
    vecFill<16>(Hrr,0);
    vecFill<4>(bt,0);
    vecFill<4>(br,0);

    if (referenceIndex >=0  && currentIndex >=0){
      retval = context->processCorrespondence(&error, Htt, Hrr, Htr,
					      bt, br,
					      referenceIndex, 
					      currentIndex);
    }

    __syncthreads();
    int offset = i*56;
    float* dest = context->_accumulationBuffer+offset;
    vecCopy<16>(dest,Htt);
    vecCopy<16>(dest+16,Htr);
    vecCopy<16>(dest+32,Hrr);
    vecCopy<4>(dest+48,bt);
    vecCopy<4>(dest+52,br);
  }
  __syncthreads();
};


__host__ AlignerStatus AlignerContext::simpleIteration(int* referenceIndices, int* currentIndices, float* transform){

  // copy the reference indices and the current indices in the context buffers
  _referenceIndices.map(_rows,_cols,referenceIndices);
  cudaError_t err ;
  err = cudaMemcpy(_cudaHostContext->_referenceIndices.values(), _referenceIndices.values(), _rows*_cols*sizeof(int), cudaMemcpyHostToDevice);
  if (err!=cudaSuccess)
    return AlignerStatus(CopyToDevice,Reference, Indices);

  _currentIndices.map(_rows,_cols,currentIndices);
  err = cudaMemcpy(_cudaHostContext->_currentIndices.values(), _currentIndices.values(), _rows*_cols*sizeof(int), cudaMemcpyHostToDevice);
  if (err!=cudaSuccess)
    return AlignerStatus(CopyToDevice,Current, Indices);


  struct timeval tvStart;
  gettimeofday(&tvStart,0);

  vecCopy<16>(_cudaHostContext->_transform, transform);
  _cudaHostContext->_checksum = 0;
  err = cudaMemcpy(_cudaDeviceContext,_cudaHostContext, sizeof(AlignerContext), cudaMemcpyHostToDevice);
  if (err!=cudaSuccess)
    return AlignerStatus(CopyToDevice,Context);

  
  
  int numElements = _rows * _cols;
  int threadsPerBlock = 256;
  int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
  printf("CUDA kernel launch with %d blocks of %d threads\n", blocksPerGrid, threadsPerBlock);
  Aligner_simpleIterationKernel<<<blocksPerGrid,threadsPerBlock>>>(_cudaDeviceContext); 
  err = cudaGetLastError();
  if (err != cudaSuccess)
    {
       return AlignerStatus(KernelLaunch);
    }

  struct timeval tvEnd;
  gettimeofday(&tvEnd,0);

  const int bufferElement = 16*3+8;
  const int bsize = bufferElement*_rows*_cols;
  err = cudaMemcpy(_accumulationBuffer, _cudaHostContext->_accumulationBuffer, bsize*sizeof(float), cudaMemcpyDeviceToHost);
  if (err !=cudaSuccess)
    return AlignerStatus(CopyFromDevice,AccumulationBuffer);


  err = cudaMemcpy(_cudaHostContext,_cudaDeviceContext, sizeof(AlignerContext), cudaMemcpyDeviceToHost);
  if (err!=cudaSuccess)
    return AlignerStatus(CopyFromDevice,Context);
  printf("$$$$$$$$$$$$$$$$$$$$$checksum: %d\n", _cudaHostContext->_checksum);
  fflush(stdout);
  double tStart = tvStart.tv_sec*1000+tvStart.tv_usec*0.001;
  double tEnd = tvEnd.tv_sec*1000+tvEnd.tv_usec*0.001;

  printf("\n@@@@@@@@@@@@@@@@@@@@time: %lf",tEnd-tStart);

  return AlignerStatus();
}
// frees the cuda context

/*
__device__ void Aligner_projectPoint(const AlignerContext* context, const float* point){
  float ip[4];
  matVecMul<4,4>(ip, context->KT, point);
  float iw=1./ip[2];
  int x = iw*ip[0];
  int y = iw*ip[1];
  int d = 1000*ip[2];
  int pixelPos =context->referenceDepths.rows*y + x;
  if (d<0 || d > context->maxDepth || pixelPos > context->referenceDepths.rows*context->referenceDepths.cols)
    return;
  atomicMin((context->referenceDepths.values+pixelPos),d);
}
*/


__device__ int AlignerContext::processCorrespondence(float* error,
						     float* Htt,
						     float* Hrr,
						     float* Htr,
						     float* bt,
						     float* br,
						     int referenceIndex, int currentIndex){


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

  // vecCopy<4>(Htt,referencePoint);
  // vecCopy<4>(Htt+4,referenceNormal);
  // vecCopy<4>(Htt+8,currentPoint);
  // vecCopy<4>(Htt+12,currentNormal);

  // vecFill<16>(Htr,pointsDistance);
  // vecFill<16>(Hrr,curvatureRatio);
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

  //matPrint(ep,4,1,"ep");
  //matPrint(en,4,1,"en");

  //float localError = pointError.dot(ep) + normalError.dot(en);
  
  float localError = vecDot<4>(ep,pointsDifference) + vecDot<4>(en,normalsDifference);

  int chiOk = localError < _inlierThreshold;
  if (! chiOk)
    return 0;
  
  //Matrix4f Sp = skew(referencePoint);
  float Sp[16];
  matBuildSkew(Sp,referencePoint);
  // compute the transposed (for the skew is mult by -1)

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
    return context->initComputation(referencePointsPtr, 
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
				int* referenceIndices,
				int* currentIndices,
				float* transform){
    return context->simpleIteration(referenceIndices, currentIndices, transform);
  }

  int AlignerContext::getHb(float* Htt, float* Htr, float* Hrr, float*bt, float* br){
    vecFill<16>(Htt,0);
    vecFill<16>(Htr,0);
    vecFill<16>(Hrr,0);
    vecFill<16>(bt,0);
    vecFill<16>(br,0);
    float* bptr = _accumulationBuffer;
    for (int i=0; i<_rows*_cols; i++){
      vecSum<16>(Htt,bptr,1.0f); bptr += 16;
      vecSum<16>(Htr,bptr,1.0f); bptr += 16;
      vecSum<16>(Hrr,bptr,1.0f); bptr += 16;
      vecSum<4>(bt,bptr,1.0f);   bptr += 4;
      vecSum<4>(br,bptr,1.0f);   bptr += 4;
    }
    return _rows*_cols;
  }


  int getHb(AlignerContext* context, float* Htt, float* Hrt, float* Hrr, float*bt, float* br){
    return context->getHb(Htt, Hrt, Hrr, bt, br);
  }
} // end namespace


#ifdef __TEST
int main(int argc, char**argv){
  AlignerContext context;
  AlignerStatus status;
  printf("Initialization\n");
  status = context.init(640*480,640*480, 640, 480);
  char buf [1024];
  status.toString(buf);
  printf("%s\n",buf);

  printf("Termination\n");
  status = context.free();
  status.toString(buf);
  printf("%s\n",buf);
}
#endif
