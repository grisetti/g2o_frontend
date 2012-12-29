#ifndef _G2O_FRONTEND_SE2_ALIGNMENT_
#define _G2O_FRONTEND_SE2_ALIGNMENT_
#include "ransac.h"
#include "g2o/types/slam2d/types_slam2d.h"

namespace g2o_frontend{

  typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexSE2>     AlignmentAlgorithmSE2SE2;

  class AlignmentAlgorithmSE2: public AlignmentAlgorithmSE2SE2 {
  public:
    AlignmentAlgorithmSE2();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };

  class RansacSE2    : public GeneralizedRansac<AlignmentAlgorithmSE2> {
    RansacSE2();
  };

}
#endif
