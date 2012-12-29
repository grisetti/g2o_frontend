#include "alignment_se2.h"
#include <Eigen/SVD>
#include <iostream>

namespace g2o_frontend{
  using namespace g2o;
  using namespace Eigen;
  using namespace std;

  AlignmentAlgorithmSE2::AlignmentAlgorithmSE2(): AlignmentAlgorithmSE2SE2(1) {
  }

  bool AlignmentAlgorithmSE2::operator()(AlignmentAlgorithmSE2::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
    if ((int)indices.size()<minimalSetSize())
      return false;

    SE2 x0;
    SE2 ix0;
    double deltaRSum=0.;
    Vector2d mean1(0.,0.);
    Vector2d mean2(0.,0.);
    for (size_t i=0; i<indices.size(); i++){
      const Correspondence& c = correspondences[indices[i]];
      const EdgeSE2* edge = static_cast<const g2o::EdgeSE2*>(c.edge());
      const VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(edge->vertex(0));
      const VertexSE2* v2 = static_cast<const g2o::VertexSE2*>(edge->vertex(1));
      SE2 xi = v2->estimate()*v1->estimate().inverse();
      mean1 += v1->estimate().translation();
      mean2 += v2->estimate().translation();
      if (i==0){
	x0 = xi;
	ix0 = x0.inverse();
      } else {
	SE2 delta=ix0*xi;
	deltaRSum += delta.rotation().angle();
      }
    }
    int count  = indices.size();
    double icount = 1./double(count);
    deltaRSum*=icount;
    mean1*=icount;
    mean2*=icount;
    Rotation2Dd R = x0.rotation()*Rotation2Dd(deltaRSum);
    transform.setRotation(R);
    transform.setTranslation(mean2 - R*mean1);
    transform = transform.inverse();
    return true;
  }


  RansacSE2::RansacSE2(): GeneralizedRansac<AlignmentAlgorithmSE2>(1){}
}
