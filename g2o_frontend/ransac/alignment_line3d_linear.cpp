#include "g2o_frontend/dm_optimization/dm_math.h"
#include "alignment_line3d_linear.h"
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <iostream>
using namespace std;

namespace g2o_frontend{
  using namespace g2o;
  using namespace Slam3dAddons;
  using namespace Eigen;

  inline void _skew(Eigen::Matrix3d& S, const Eigen::Vector3d& t){
    S <<   
      0,  -t.z(),   t.y(),
      t.z(),     0,     -t.x(),
      -t.y()     ,t.x(),   0;
  }

  inline Eigen::Matrix3d _skew(const Eigen::Vector3d& t){
    Eigen::Matrix3d S;
    S <<   
      0,  -t.z(),   t.y(),
      t.z(),     0,     -t.x(),
      -t.y(),     t.x(),   0;
    return S;
  }



  AlignmentAlgorithmLine3DLinear::AlignmentAlgorithmLine3DLinear(): AlignmentAlgorithmSE3Line3D(2) {
  }
  
  bool AlignmentAlgorithmLine3DLinear::operator()(AlignmentAlgorithmLine3DLinear::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
    if ((int)indices.size()<minimalSetSize())
      return false;
    transform = Isometry3d::Identity();
    Vector12d x=homogeneous2vector(transform.matrix());
        
    Matrix12d H;
    H.setZero();
    Vector12d b;
    b.setZero();
    Matrix6x12d A;
    for (size_t i=0; i<indices.size(); i++){
      A.setZero();
      const Correspondence& c = correspondences[indices[i]];
      const EdgeLine3D* edge = static_cast<const EdgeLine3D*>(c.edge()); 
      const VertexLine3D* v1 = static_cast<const VertexLine3D*>(edge->vertex(0));
      const VertexLine3D* v2 = static_cast<const VertexLine3D*>(edge->vertex(1));
      const AlignmentAlgorithmLine3DLinear::PointEstimateType& li= v1->estimate();
      const AlignmentAlgorithmLine3DLinear::PointEstimateType& lj= v2->estimate();
      A.block<1,3>(0,0)=lj.w().transpose();
      A.block<1,3>(1,3)=lj.w().transpose();
      A.block<1,3>(2,6)=lj.w().transpose();
      A.block<3,3>(0,9)=-_skew(li.d());
      A.block<1,3>(3,0)=lj.d().transpose();
      A.block<1,3>(4,3)=lj.d().transpose();
      A.block<1,3>(5,6)=lj.d().transpose();
      Vector6d ek = li;
      ek -= A*x;
      H+=A.transpose()*A;
      b+=A.transpose()*ek;
    }
    LDLT<Matrix12d> ldlt(H);
    if (ldlt.isNegative())
      return false;
    x=ldlt.solve(b); // using a LDLT factorizationldlt;
    
    Matrix4d _X = transform.matrix()+vector2homogeneous(x);
    
    // recondition the rotation 
    JacobiSVD<Matrix3d> svd(_X.block<3,3>(0,0), Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues()(0)<.5)
      return false;
    Matrix3d R=svd.matrixU()*svd.matrixV().transpose();
    Isometry3d X = Isometry3d::Identity();
    X.linear()=R;
    X.translation() = _X.block<3,1>(0,3);

    Matrix3d H2=Matrix3d::Zero();
    Vector3d b2=Vector3d::Zero();

    for (size_t i=0; i<indices.size(); i++){
      const Correspondence& c = correspondences[indices[i]];
      const EdgeLine3D* edge = static_cast<const EdgeLine3D*>(c.edge()); 
      const VertexLine3D* v1 = static_cast<const VertexLine3D*>(edge->vertex(0));
      const VertexLine3D* v2 = static_cast<const VertexLine3D*>(edge->vertex(1));
      const AlignmentAlgorithmLine3DLinear::PointEstimateType& li= v1->estimate();
      const AlignmentAlgorithmLine3DLinear::PointEstimateType& lj= v2->estimate();
      Matrix3d A2=-_skew(R*lj.d());
      Vector3d ek = li.w()-R*lj.w()-A2*X.translation();
      H2+=A2.transpose()*A2;
      b2+=A2.transpose()*ek;
    }
    Vector3d dt;
    dt = H2.ldlt().solve(b2);
    X.translation()+=dt;
    transform = X;
    cerr << "transform: " << endl;
    cerr << g2o::internal::toVectorMQT(transform) << endl;;
    return true;
  }


  RansacLine3DLinear::RansacLine3DLinear():  GeneralizedRansac<AlignmentAlgorithmLine3DLinear>(2){
  }

}
