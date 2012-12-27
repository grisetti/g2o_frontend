#ifndef _G2O_FRONTEND_HORN_ALIGNMENT_
#define _G2O_FRONTEND_HORN_ALIGNMENT_
#include "ransac.h"

namespace g2o_frontend{
  class AlignmentAlgorithmHorn2D: public AlignmentAlgorithmSE2PointXY {
  public:
    AlignmentAlgorithmHorn2D();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };


}
#endif
