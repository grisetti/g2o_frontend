#ifndef _ALIGNER_CUDA_H_
#define _ALIGNER_CUDA_H_
#include <assert.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include "cudaptr.cuh"
#include "cudamatrix.cuh"


struct IntMatrix {
  int rows;
  int cols;
  int* values;
};

template <int rows_>
struct FloatMatrix{
  static const int rows = rows_;
    //int rows;
  int cols;
  float* values; 
};

struct AlignerContext {
  // to be passed only once
  FloatMatrix<4> referencePoints;
  FloatMatrix<4> referenceNormals;
  float* referenceCurvatures;
  FloatMatrix<4> currentPoints;
  FloatMatrix<4> currentNormals;
  FloatMatrix<16> currentOmegaPs;
  FloatMatrix<16> currentOmegaNs;
  float* currentCurvatures;
  
  // to be passed once per iteration
  IntMatrix   currentIndices;
  IntMatrix   referenceIndices;
  IntMatrix   referenceDepths;
  // cuda temporaries for the reduce;

 // parameters
  float distanceThreshold;
  float normalThreshold;
  float flatCurvatureThreshold;
  float minCurvatureRatio;
  float maxCurvatureRatio;
  float inlierThreshold;
  float maxDepth;
  float transform[16];
  float KT[16];

};


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
#endif
