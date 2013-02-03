#ifndef _G2O_FRONTEND_ALIGNMENT_LINE2D_LINEAR_H
#define _G2O_FRONTEND_ALIGNMENT_LINE2D_LINEAR_H

#include "ransac.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"

namespace g2o_frontend{
	
	typedef AlignmentAlgorithm<Eigen::Isometry2d,g2o::VertexLine2D>   AlignmentAlgorithmSE2Line2D;
	
	class AlignmentAlgorithmLine2DLinear : public AlignmentAlgorithmSE2Line2D {
	public:
		AlignmentAlgorithmLine2DLinear();
		virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices);
		
	};
	
  class RansacLine2DLinear: public GeneralizedRansac<AlignmentAlgorithmLine2DLinear>{
  public:
    RansacLine2DLinear();
  };
	
}
#endif // ALIGNMENT_LINE2D_LINEAR_H
