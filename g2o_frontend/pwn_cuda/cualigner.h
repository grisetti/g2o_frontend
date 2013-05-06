#ifndef _CU_ALIGNER_
#define _CU_ALIGNER_
#include "g2o_frontend/pwn/aligner.h"

namespace CudaAligner {
  struct AlignerContext;
  class CuAligner: public Aligner {
  public:
    void align();
  };
};

#endif
