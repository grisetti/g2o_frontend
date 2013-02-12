#include "g2o_frontend/basemath/bm_se3.h"
#include "alignment_plane3d_linear.h"
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <iostream>
using namespace std;

namespace g2o_frontend{
  using namespace g2o;
  using namespace Slam3dAddons;
  using namespace Eigen;

  AlignmentAlgorithmPlane3DLinear::AlignmentAlgorithmPlane3DLinear(): AlignmentAlgorithmSE3Plane3D(2) {
  }
  
  bool AlignmentAlgorithmPlane3DLinear::operator()(AlignmentAlgorithmPlane3DLinear::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
    
    return true;
  }


  RansacPlane3DLinear::RansacPlane3DLinear():  GeneralizedRansac<AlignmentAlgorithmPlane3DLinear>(2){
  }

}
