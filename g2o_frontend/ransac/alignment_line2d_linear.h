#ifndef _G2O_FRONTEND_ALIGNMENT_LINE2D_LINEAR_H
#define _G2O_FRONTEND_ALIGNMENT_LINE2D_LINEAR_H

#include "ransac.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"

namespace g2o_frontend{
	
  typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexLine2D>   AlignmentAlgorithmSE2Line2D;
	
  Eigen::Vector3d line2d_remapCartesian(const Eigen::Isometry2d& _X, Eigen::Vector3d& _l){
    
    Eigen::Vector3d tl = _l;
    Eigen::Matrix3d X = _X.matrix();
    tl.head<2>() = X.block<2,2>(0,0) * _l.block<2,1>(0,0);
    tl[2] += X.block<2,1>(0,2).transpose() * tl.head<2>();
		
    return tl;
  }
	
  class AlignmentAlgorithmLine2DLinear : public AlignmentAlgorithmSE2Line2D {
  public:
    AlignmentAlgorithmLine2DLinear();
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices);
		
  private:
    void Zero();
  };
	
  class RansacLine2DLinear: public GeneralizedRansac<AlignmentAlgorithmLine2DLinear>{
  public:
    RansacLine2DLinear();
  };
	
}
#endif // ALIGNMENT_LINE2D_LINEAR_H
