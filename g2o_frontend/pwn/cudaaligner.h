#ifndef _ALIGNER_CUDA_H_
#define _ALIGNER_CUDA_H_
#include <assert.h>
#include <cstdio>
#include <cuda.h>
#include "cudamatrix.cuh"

struct AlignerContext {

  enum Operation {
    Ok=0x0, 
    Allocation=0x1, 
    Deallocation=0x2, 
    CopyToDevice=0x3, 
    CopyFromDevice=0x4, 
    KernelLaunch=0x5, 
    InitComputation=0x6
  };


  enum Object    {
    NoObject=0x0, 
    Reference=0x1, 
    Current=0x2, 
    Context=0x3, 
    DepthBuffer=0x4, 
    AccumulationBuffer=0x5
  };



  enum ObjectDetail  {
    NoDetail=0x0, 
    Points=0x1, 
    Normals=0x2, 
    Curvatures=0x3, 
    OmegaPs=0x4, 
    OmegaNs=0x5, 
    Indices=0x6
  };

  static const char* strOperation[];
  static const char* strObject[];
  static const char* strObjectDetail[];


  struct AlignerStatus{
    AlignerStatus(Operation op=Ok, Object object=NoObject, ObjectDetail detail=NoDetail){
      _operation=op;
      _object=object;
      _detail= detail;
    }
    Operation _operation;
    Object    _object;
    ObjectDetail _detail;
    void toString(char* s);
  };
  
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
  float _maxDepth;
  float _transform[16];
  float _KT[16];

    // initializes the default values and sets the base parameters
  AlignerStatus init(int maxReferencePoints, int maxCurrentPoints, int rows, int cols);

  // initializes the computation by passing all the values that will not change during the iterations
  AlignerStatus initComputation(float* referencePointsPtr, 
				float* referenceNormalsPtr, 
				float* referenceCurvaturesPtr, 
				int numReferencePoints_, 
				float* currentPointsPtr, 
				float* currentNormalsPtr, 
				float* currentCurvaturesPtr, 
				float* currentOmegaPsPtr, 
				float* currentOmegaNPtr, 
				int numCurrentPoints_);

  AlignerStatus simpleIteration(int* referenceIndices, int* currentIndices, float* transform);

  // frees the cuda context
  AlignerStatus free();


// private cuda context
//private:
  AlignerContext* _cudaDeviceContext, *_cudaHostContext ;
  float* _accumulationBuffer;
  __device__ inline int processCorrespondence(float* error,
					      float* Htt,
					      float* Hrr,
					      float* Htr,
					      float* bt,
					      float* br,
					      int referenceIndex, int currentIndex);
  
};
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
