#ifndef _ALIGNER_CUDA_CUH_
#define _ALIGNER_CUDA_CUH_
#include <assert.h>
#include <cstdio>
#include <cuda.h>
#include "cudaaligner.h"
#include "cudamatrix.cuh"


namespace pwn {
  
struct AlignerContext {
  
  int _maxReferencePoints, _maxCurrentPoints;
  int _numReferencePoints, _numCurrentPoints;
  int _rows, _cols;

  // to be passed only once
  FloatMatrix4N _referencePoints;
  FloatMatrix4N _referenceNormals;
  float* _referenceCurvatures;
  FloatMatrix4N _currentPoints;
  FloatMatrix4N _currentNormals;
  FloatMatrix16N _currentOmegaPs;
  FloatMatrix16N _currentOmegaNs;
  float* _currentCurvatures;
  
  
  // to be passed once per iteration
  IntMatrix   _currentIndices;
  IntMatrix   _referenceIndices;
  IntMatrix   _depthBuffer;
  // cuda temporaries for the reduce;


 // parameters
  float _distanceThreshold;
  float _normalThreshold;
  float _flatCurvatureThreshold;
  float _minCurvatureRatio;
  float _maxCurvatureRatio;
  float _inlierThreshold;
  int _maxDepth;
  int _minDepth;
  float _transform[16];
  float _sensorOffset[16];
  float _cameraMatrix[16];
  float _Hb[56];
  float _KT[16];
  float _KOT[16];
  float _KO[16];
  bool _robustKernel;
  int _checksum;
  __host__ int inliers() const {return _inliers;}
  __host__ float error() const {return _error;}
  __host__ float time() const {return _time;}
    // initializes the default values and sets the base parameters
  __host__ AlignerStatus init(int maxReferencePoints, int maxCurrentPoints, int rows, int cols);

  // initializes the computation by passing all the values that will not change during the iterations
  __host__ AlignerStatus initComputation(const float* cameraMatrix,
					 const float* sensorOffset,
					 float* referencePointsPtr, 
					 float* referenceNormalsPtr, 
					 float* referenceCurvaturesPtr, 
					 int numReferencePoints_, 
					 float* currentPointsPtr, 
					 float* currentNormalsPtr, 
					 float* currentCurvaturesPtr, 
					 float* currentOmegaPsPtr, 
					 float* currentOmegaNPtr, 
					 int numCurrentPoints_);

  AlignerStatus simpleIteration(float* transform);

  // frees the cuda context
  __host__ AlignerStatus free();


// private cuda context
//private:
  AlignerContext* _cudaDeviceContext, *_cudaHostContext ;
  float* _accumulationBuffer;
  float* _reductionBuffer;
  __device__ inline int processCorrespondence(float* error,
				   float* Htt,
				   float* Hrr,
				   float* Htr,
				   float* bt,
				   float* br,
				   int referenceIndex, int currentIndex);
  
  int getHb(float* Htt, float* Hrt, float* Hrr, float*bt, float* br);
  float _error;
  int _inliers;
  double _time;
};

}// end namespace
/*

void matPrint(const float* m, int r, int c, const char* msg=0);


void Aligner_fillContext(AlignerContext* context,
			 float* referencePointsPtr, float* referenceNormalsPtr, float* referenceCurvatres, 
			 int numReferencePoints, 
			 float* currentPointsPtr, float* currentNormalsPtr, float* currentCurvatures, 
			 float* currentOmegaPsPtr, float* currentOmegaNsPtr,
			 int numCurrentPoints,
			 
			 int* referenceIndicesPtr,
			 int* currentIndicesPtr,
			 int imageRows,
			 int imageCols,
			 float* transform);

int Aligner_processCorrespondences(float* globalError,
				   float* Htt_,
				   float* Htr_,
				   float* Hrr_,
				   float* bt_,
				   float* br_, const AlignerContext* context);
*/
#endif
