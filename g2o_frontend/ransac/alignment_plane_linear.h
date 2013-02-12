#ifndef _G2O_FRONTEND_ALIGNMENT_PLANE_3D_
#define _G2O_FRONTEND_ALIGNMENT_PLANE_3D_
#include "ransac.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

namespace g2o_frontend{

  typedef AlignmentAlgorithm<Eigen::Isometry3d,Slam3dAddons::VertexPlane>   AlignmentAlgorithmSE3Plane3D;

  class AlignmentAlgorithmPlane3DLinear: public AlignmentAlgorithmSE3Plane3D {
  public:
    AlignmentAlgorithmPlane3DLinear();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices); 

  };
  
  class RansacPlane3DLinear: public GeneralizedRansac<AlignmentAlgorithmPlane3DLinear>{
  public:
    RansacPlane3DLinear();
  };


}
#endif
