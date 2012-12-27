#include "alignment_horn3d.h"
#include <Eigen/SVD>

namespace g2o_frontend{
  using namespace g2o;
  using namespace Eigen;

  AlignmentAlgorithmHorn3D::AlignmentAlgorithmHorn3D(): AlignmentAlgorithmSE3PointXYZ(3) {
  }

  bool AlignmentAlgorithmHorn3D::operator()(AlignmentAlgorithmHorn3D::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
    if ((int)indices.size()<minimalSetSize())
      return false;
    Eigen::Vector3d mean1=Eigen::Vector3d::Zero();
    Eigen::Vector3d mean2=Eigen::Vector3d::Zero();

    for (size_t i=0; i<indices.size(); i++){
      const Correspondence& c = correspondences[indices[i]];
      const EdgePointXYZ* edge = static_cast<const g2o::EdgePointXYZ*>(c.edge());
      const VertexPointXYZ* v1 = static_cast<const g2o::VertexPointXYZ*>(edge->vertex(0));
      const VertexPointXYZ* v2 = static_cast<const g2o::VertexPointXYZ*>(edge->vertex(1));
      mean1 +=v1->estimate();
      mean2 +=v2->estimate();
    }
    int count  = indices.size();
    double icount = 1./double(count);
    mean1*=icount;
    mean2*=icount;

    Eigen::Matrix3d M=Eigen::Matrix3d::Zero();
    for (size_t i=0; i<indices.size(); i++){
      const Correspondence& c = correspondences[indices[i]];
      const EdgePointXYZ* edge = static_cast<const g2o::EdgePointXYZ*>(c.edge());
      const VertexPointXYZ* v1 = static_cast<const g2o::VertexPointXYZ*>(edge->vertex(0));
      const VertexPointXYZ* v2 = static_cast<const g2o::VertexPointXYZ*>(edge->vertex(1));
      PointEstimateType p1 =v1->estimate()-mean1;
      PointEstimateType p2 =v2->estimate()-mean2;
      M+=p1*p2.transpose();
    }
    
    JacobiSVD<Matrix3d> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double svdThreshold = 1e-2;
    if (svd.singularValues()(1)/svd.singularValues()(0)<svdThreshold &&
	svd.singularValues()(2)/svd.singularValues()(1)<svdThreshold)
      return false; 

    Matrix3d R=svd.matrixU()*svd.matrixV().transpose();
    transform.linear()=R;
    transform.translation()= mean1 - R*mean2;
    return true;
  }
  


}
