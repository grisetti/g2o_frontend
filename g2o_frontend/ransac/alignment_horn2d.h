#ifndef _G2O_FRONTEND_HORN_ALIGNMENT_2D_
#define _G2O_FRONTEND_HORN_ALIGNMENT_2D_
#include "g2o/types/slam2d/types_slam2d.h"
#include "distance_correspondence_validator.h"
#include "ransac.h"

namespace g2o_frontend{

  typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexPointXY> AlignmentAlgorithmSE2PointXY;


  class AlignmentAlgorithmHorn2D: public AlignmentAlgorithmSE2PointXY {
  public:
    AlignmentAlgorithmHorn2D();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };

  class DistanceCorrespondenceValidatorPointXY: public DistanceCorrespondenceValidator<g2o::VertexPointXY>{
  public:
    DistanceCorrespondenceValidatorPointXY();
  };

  class RansacHorn2D: public GeneralizedRansac<AlignmentAlgorithmHorn2D> {
    RansacHorn2D();
  };


}
#endif
