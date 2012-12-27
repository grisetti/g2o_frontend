#include "alignment_horn2d.h"
#include <Eigen/SVD>

namespace g2o_frontend{
  using namespace g2o;
  using namespace Eigen;

  AlignmentAlgorithmHorn2D::AlignmentAlgorithmHorn2D(): AlignmentAlgorithmSE2PointXY(2) {
  }

  bool AlignmentAlgorithmHorn2D::operator()(AlignmentAlgorithmHorn2D::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
    if ((int)indices.size()<minimalSetSize())
      return false;
    Eigen::Vector2d mean1=Eigen::Vector2d::Zero();
    Eigen::Vector2d mean2=Eigen::Vector2d::Zero();

    for (size_t i=0; i<indices.size(); i++){
      const Correspondence& c = correspondences[indices[i]];
      const EdgeSE2* edge = static_cast<const g2o::EdgeSE2*>(c.edge());
      const VertexPointXY* v1 = static_cast<const g2o::VertexPointXY*>(edge->vertex(0));
      const VertexPointXY* v2 = static_cast<const g2o::VertexPointXY*>(edge->vertex(1));
      mean1 +=v1->estimate();
      mean2 +=v2->estimate();
    }
    int count  = indices.size();
    double icount = 1./double(count);
    mean1*=icount;
    mean2*=icount;

    Eigen::Matrix2d M=Eigen::Matrix2d::Zero();
    for (size_t i=0; i<indices.size(); i++){
      const Correspondence& c = correspondences[indices[i]];
      const EdgeSE2* edge = static_cast<const g2o::EdgeSE2*>(c.edge());
      const VertexPointXY* v1 = static_cast<const g2o::VertexPointXY*>(edge->vertex(0));
      const VertexPointXY* v2 = static_cast<const g2o::VertexPointXY*>(edge->vertex(1));
      PointEstimateType p1 =v1->estimate()-mean1;
      PointEstimateType p2 =v2->estimate()-mean2;
      M+=p1*p2.transpose();
    }
    
    JacobiSVD<Matrix2d> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double svdThreshold = 1e-2;
    if (svd.singularValues()(1)/svd.singularValues()(0)<svdThreshold)
      return false; 
      
    Eigen::Rotation2Dd R(0.);
    R.fromRotationMatrix(svd.matrixU()*svd.matrixV().transpose());
    transform.setRotation(R);
    transform.setTranslation(mean1 - R*mean2);
    return true;
  }
  

  // AlignmentAlgorithmHorn3D::AlignmentAlgorithmHorn3D(): AlignmentAlgorithmSE3PointXYZ(3){
  // }

  // bool AlignmentAlgorithmHorn3D::operator()(AlignmentAlgorithmHorn3D::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
  //   if (indices.size()<minimalSetSize())
  //     return false;
  // }


}
