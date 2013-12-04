#ifndef _CU_ALIGNER_
#define _CU_ALIGNER_
#include "g2o_frontend/pwn_core/aligner.h"

namespace pwn {
 
  struct AlignerContext;
  class CuAligner: public Aligner {
  public:
    CuAligner();
    void align();
    virtual ~CuAligner();
  protected:
    AlignerContext* _context;
  };
};


#endif
