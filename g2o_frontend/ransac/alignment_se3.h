#ifndef _G2O_FRONTEND_SE3_ALIGNMENT_
#define _G2O_FRONTEND_SE3_ALIGNMENT_
#include "ransac.h"

namespace g2o_frontend{
  class AlignmentAlgorithmSE3: public AlignmentAlgorithmSE3SE3 {
  public:
    AlignmentAlgorithmSE3();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };


}
#endif
