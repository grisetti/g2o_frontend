#ifndef _G2O_FRONTEND_HORN_ALIGNMENT_3D_
#define _G2O_FRONTEND_HORN_ALIGNMENT_3D_
#include "g2o/types/slam3d/types_slam3d.h"
#include "distance_correspondence_validator.h"
#include "ransac.h"

namespace g2o_frontend{
  typedef AlignmentAlgorithm<Eigen::Isometry3d,g2o::VertexPointXYZ> AlignmentAlgorithmSE3PointXYZ;

  class AlignmentAlgorithmHorn3D: public AlignmentAlgorithmSE3PointXYZ {
  public:
    AlignmentAlgorithmHorn3D();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };


  class DistanceCorrespondenceValidatorPointXYZ: public DistanceCorrespondenceValidator<g2o::VertexPointXYZ>{
  public:
    DistanceCorrespondenceValidatorPointXYZ();
  };

  class RansacHorn3D: public GeneralizedRansac<AlignmentAlgorithmHorn3D> {
  public:
    RansacHorn3D();
  };


}
#endif
