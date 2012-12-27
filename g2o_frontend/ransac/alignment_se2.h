#ifndef _G2O_FRONTEND_SE2_ALIGNMENT_
#define _G2O_FRONTEND_SE2_ALIGNMENT_
#include "ransac.h"

namespace g2o_frontend{
  class AlignmentAlgorithmSE2: public AlignmentAlgorithmSE2SE2 {
  public:
    AlignmentAlgorithmSE2();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };


}
#endif
