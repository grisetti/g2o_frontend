#ifndef _ALIGNER_CUDA_H_
#define _ALIGNER_CUDA_H_
#include <assert.h>
#include <cstdio>
#include <cuda.h>

namespace CudaAligner {

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

  struct AlignerStatus{
    static const char* strOperation[];
    static const char* strObject[];
    static const char* strObjectDetail[];
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

  struct AlignerContext;
  
  AlignerStatus createContext(AlignerContext** context, int maxReferencePoints, int maxCurrentPoints, int rows, int cols);

  AlignerStatus destroyContext(AlignerContext* context);

  AlignerStatus initComputation(AlignerContext* context,
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

  AlignerStatus simpleIteration(AlignerContext* context,
				int* referenceIndices,
				int* currentIndices,
				float* transform);

  int getHb(AlignerContext* context, float* Htt, float* Hrt, float* Hrr, float*bt, float* br);
				    
}
#endif
