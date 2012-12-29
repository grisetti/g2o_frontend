#ifndef _G2O_FRONTEND_SE3_ALIGNMENT_
#define _G2O_FRONTEND_SE3_ALIGNMENT_
#include "ransac.h"
#include "g2o/types/slam3d/types_slam3d.h"

namespace g2o_frontend{
  typedef AlignmentAlgorithm<Eigen::Isometry3d,g2o::VertexSE3>      AlignmentAlgorithmSE3SE3;

  class AlignmentAlgorithmSE3: public AlignmentAlgorithmSE3SE3 {
  public:
    AlignmentAlgorithmSE3();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };

  class RansacSE3    : public GeneralizedRansac<AlignmentAlgorithmSE3SE3> {
    RansacSE3();
  };


}
#endif
