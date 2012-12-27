#ifndef _G2O_FRONTEND_HORN_ALIGNMENT_3D_
#define _G2O_FRONTEND_HORN_ALIGNMENT_3D_
#include "ransac.h"

namespace g2o_frontend{
  class AlignmentAlgorithmHorn3D: public AlignmentAlgorithmSE3PointXYZ {
  public:
    AlignmentAlgorithmHorn3D();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };


}
#endif
