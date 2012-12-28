#ifndef _G2O_FRONTEND_ALIGNMENT_LINE_3D_
#define _G2O_FRONTEND_ALIGNMENT_LINE_3D_
#include "ransac.h"

namespace g2o_frontend{
  class AlignmentAlgorithmLine3DLinear: public AlignmentAlgorithmSE3Line3D {
  public:
    AlignmentAlgorithmLine3DLinear();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };


}
#endif
