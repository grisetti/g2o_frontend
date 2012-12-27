#include "alignment_se3.h"
#include <Eigen/Geometry>
#include <iostream>

namespace g2o_frontend{
  using namespace g2o;
  using namespace Eigen;
  using namespace std;

  AlignmentAlgorithmSE3::AlignmentAlgorithmSE3(): AlignmentAlgorithmSE3SE3(1) {
  }

  bool AlignmentAlgorithmSE3::operator()(AlignmentAlgorithmSE3::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
    if ((int)indices.size()<minimalSetSize())
      return false;

    Isometry3d x0=Isometry3d::Identity();
    Isometry3d ix0=Isometry3d::Identity();
    Vector3d mean1(0.,0.,0.);
    Vector3d mean2(0.,0.,0.);
    Vector3d deltaRSum(0.,0.,0.);
    for (size_t i=0; i<indices.size(); i++){
      const Correspondence& c = correspondences[indices[i]];
      const EdgeSE3* edge = static_cast<const g2o::EdgeSE3*>(c.edge());
      const VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(edge->vertex(0));
      const VertexSE3* v2 = static_cast<const g2o::VertexSE3*>(edge->vertex(1));
      Isometry3d xi = v2->estimate()*v1->estimate().inverse();
      mean1 += v1->estimate().translation();
      mean2 += v2->estimate().translation();
      if (i==0){
	x0 = xi;
	ix0 = x0.inverse();
      } else {
	Isometry3d delta=ix0*xi;
	AngleAxisd dR(delta.rotation());
	deltaRSum += dR.axis()*dR.angle();
      }
    }
    int count  = indices.size();
    double icount = 1./double(count);
    
    deltaRSum*=icount;
    mean1*=icount;
    mean2*=icount;
    // cerr << "mean1" << endl;
    // cerr << mean1 << endl;
    // cerr << "mean2" << endl;
    // cerr << mean2 << endl;


    AngleAxisd dR(deltaRSum.norm(), deltaRSum.normalized());
    transform.linear() = x0.linear()*dR.matrix();
    // cerr << "mean1 remapped" << endl;
    // cerr << transform.linear()*mean1 << endl;

    transform.translation() = mean2-transform.linear()*mean1;
    transform = transform.inverse();
    return true;
  }

}
