#include "cudatest.h"
#include <cstdio>
#include <fstream>
#include <omp.h>
using namespace std;

#define __device__ inline
#include <cmath>
#define atomicMin(p,v)  if ((*p) > v ) (*p) = v;

#include "cudatest.cu"

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

void Aligner_fillContext(AlignerContext* context,
			 float* referencePointsPtr,
			 float* referenceNormalsPtr,
			 float* referenceCurvatures,
			 int numReferencePoints,
			 float* currentPointsPtr,
			 float* currentNormalsPtr,
			 float* currentCurvatures,
			 float* currentOmegaPsPtr,
			 float* currentOmegaNsPtr,
			 int numCurrentPoints,
			 int* referenceIndicesPtr,
			 int* currentIndicesPtr,
			 int imageRows,
			 int imageCols,
			 float* transform
			 ){
  FloatMatrix_map(&context->referencePoints, 4, numReferencePoints, referencePointsPtr);
  FloatMatrix_map(&context->referenceNormals, 4, numReferencePoints, referenceNormalsPtr);
  FloatMatrix_map(&context->currentPoints, 4, numCurrentPoints, currentPointsPtr);
  FloatMatrix_map(&context->currentNormals, 4, numCurrentPoints, currentNormalsPtr);
  FloatMatrix_map(&context->currentOmegaPs, 16, numCurrentPoints, currentOmegaPsPtr);
  FloatMatrix_map(&context->currentOmegaNs, 16, numCurrentPoints, currentOmegaNsPtr);
  
  IntMatrix_map(&context->currentIndices, imageRows, imageCols,   currentIndicesPtr);
  IntMatrix_map(&context->referenceIndices, imageRows, imageCols, referenceIndicesPtr);
  context->referenceCurvatures = referenceCurvatures;
  context->currentCurvatures = currentCurvatures;

  vecCopy(context->transform, transform, 16);
  matPrint(transform, 4,4, "gputransform");
}



int Aligner_processCorrespondences(float* globalError,
				   float* Htt_,
				   float* Htr_,
				   float* Hrr_,
				   float* bt_,
				   float* br_, 
				   const AlignerContext* context){
  vecFill<16>(Htt_,0);
  vecFill<16>(Htr_,0);
  vecFill<16>(Hrr_,0);
  vecFill<4>(bt_,0);
  vecFill<4>(br_,0);
  

  int inliers = 0;
  *globalError=0;
  int badNormals = 0;
  int tooDistant = 0;
  int badCurvature =0;
  int noPoint = 0;
  printf ("processing a %d x %d image\n", context->currentIndices.rows, context->currentIndices.cols);
  //ofstream os ("gpucorr.txt");
  int numThreads = omp_get_max_threads();
  int colsPerThread = context->currentIndices.cols/numThreads;
  float t_Htt[numThreads][16];
  float t_Htr[numThreads][16];
  float t_Hrr[numThreads][16];
  float t_bt[numThreads][4];
  float t_br[numThreads][4];
  int   t_inliers[numThreads];
  int   t_errors[numThreads];
  #pragma omp parallel
  {
    int threadId = omp_get_thread_num();
    int cmin = threadId*colsPerThread;
    int cmax = (threadId+1)*colsPerThread;
    if (cmax > context->currentIndices.cols)
      cmax = context->currentIndices.cols;
    vecFill<16>(t_Htt[threadId],0);
    vecFill<16>(t_Htr[threadId],0);
    vecFill<16>(t_Hrr[threadId],0);
    vecFill<4>(t_bt[threadId],0);
    vecFill<4>(t_br[threadId],0);
    t_inliers[threadId] = 0;
    t_errors[threadId] = 0;
    for (int c=cmin; c<cmax ; c++) {

      int* currPtr = context->currentIndices.values + (cmin*context->currentIndices.rows);
      int* refPtr = context->referenceIndices.values + (cmin*context->currentIndices.rows);
      
      for (int r=0; r<context->currentIndices.rows; r++, refPtr++, currPtr++){
	if (*currPtr<0 || *refPtr<0){
	  //os << *refPtr  << " " << *currPtr << endl;
	  noPoint++;
	  continue;
	}
	float error;
	float Htt[16], Htr[16], Hrr[16], bt[4], br[4];
	
	int result = Aligner_processCorrespondence(&error, Htt, Hrr, Htr, bt, br,
						   *refPtr, *currPtr, context, &badNormals, &tooDistant, &badCurvature);

	
	if (result){
	  vecSum<16>(t_Htt[threadId],Htt,1.0f);
	  vecSum<16>(t_Hrr[threadId],Hrr,1.0f);
	  vecSum<16>(t_Htr[threadId],Htr,1.0f);
	  vecSum<4>(t_bt[threadId],bt,1.0f);
	  vecSum<4>(t_br[threadId],br,1.0f);
	  t_inliers[threadId] ++;
	  t_errors[threadId] += error;
	}
      }
    }
  }
  *globalError = 0;
  inliers = 0;
  for (int threadId = 0; threadId<numThreads; threadId++){
    vecSum<16>(Htt_, t_Htt[threadId],1.0f);
    vecSum<16>(Hrr_, t_Hrr[threadId],1.0f);
    vecSum<16>(Htr_, t_Htr[threadId],1.0f);
    vecSum<4>(bt_,t_bt[threadId],1.0f);
    vecSum<4>(br_,t_br[threadId],1.0f);
    inliers += t_inliers[threadId];
    *globalError += t_errors[threadId];
  }

  //os.close();
  printf ("gpu: inliers:%d, noPoint:%d, badNormals:%d, tooDistant:%d, badCurvature:%d\n",
	  inliers, noPoint, badNormals,tooDistant,badCurvature);
  return inliers;
}

/* simple vet test thing */
/*
int main() {
  float v1[4];
  float v2[4];
  for (int i=0; i<4; i++){
    v1[i]=i;
  }
  matPrint(v1,4,1, "v1");
  vecFill(v2, 2, 4);
  matPrint(v2,4,1, "v2");
  vecScale(v1,-1,4);
  matPrint(v1,4,1, "v1=v1*-1");
  printf("v1*v1 = %f\n", vecDot(v1,v1,4));
  vecSum(v1,v2,2.0,4);
  matPrint(v1,4,1, "v1+2*v2");
  vecCopy(v2,v1,4);
  matPrint(v2,4,1, "v2=v1");
  vecScale(v2,0.1,4);
  matPrint(v2,4,1, "v2*.1");
  float m[16];
  for (int i=0; i<4; i++)
    for (int j=0; j<4; j++)
      m[i*4+j]=((i+1)*(j+1))/16.0f;
  matPrint(m,4,4,"m");
  float v3[4];
  matVecMul(v3,m,v2,4,4);
  matPrint(v3,4,1,"m*v2");
  float m3[16];
  matMatMul(m3, m, m, 4,4,4);
  matPrint(m3,4,4,"m*m");
  matBuildSkew(m3,v1);
  matPrint(m3,4,4,"skew(v1)");
  float vm[4];
  matMatMul(vm,m3,v1,4,4,1);
  matPrint(vm,4,1,"non square matrix product");
  
}
*/
