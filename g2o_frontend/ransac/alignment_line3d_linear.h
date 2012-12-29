#ifndef _G2O_FRONTEND_ALIGNMENT_LINE_3D_
#define _G2O_FRONTEND_ALIGNMENT_LINE_3D_
#include "ransac.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

namespace g2o_frontend{

  typedef AlignmentAlgorithm<Eigen::Isometry3d,Slam3dAddons::VertexLine3D>   AlignmentAlgorithmSE3Line3D;

  class AlignmentAlgorithmLine3DLinear: public AlignmentAlgorithmSE3Line3D {
  public:
    AlignmentAlgorithmLine3DLinear();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };
  
  class RansacLine3DLinear: public GeneralizedRansac<AlignmentAlgorithmLine3DLinear>{
  public:
    RansacLine3DLinear();
  };


}
#endif
