#include "g2o_frontend/basemath/bm_se2.h"
#include "alignment_line2d_linear.h"
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <iostream>
using namespace std;

namespace g2o_frontend{
  using namespace g2o;
//   using namespace Slam2dAddons;
  using namespace Eigen;

  AlignmentAlgorithmLine2DLinear::AlignmentAlgorithmLine2DLinear(): AlignmentAlgorithmSE2Line2D(2) {
  }
  bool AlignmentAlgorithmLine2DLinear::operator()(AlignmentAlgorithmLine2DLinear::TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices){
    if ((int)indices.size()<minimalSetSize())
      return false;
    transform = Isometry2d::Identity();
    Vector6d _x = homogeneous2vector_2d(transform.matrix());
		Eigen::Matrix3d Omega = Eigen::Matrix3d::Identity()*1000; 
		
// 	considering only the rotational part to first compute rotational part of the transformation
		Vector4d x;
		x.head<4>() = _x.head<4>();

    Matrix4d H;
    H.setZero();
    Vector4d b;
    b.setZero();
    Matrix2x4d A;
		//to compute the error
		double err = 0;
		
    for (size_t i=0; i<indices.size(); i++){
      A.setZero();
      const Correspondence& c = correspondences[indices[i]];
			const EdgeLine2D* edge = static_cast<const EdgeLine2D*>(c.edge());
      const VertexLine2D* v1 = static_cast<const VertexLine2D*>(edge->vertex(0));
      const VertexLine2D* v2 = static_cast<const VertexLine2D*>(edge->vertex(1));
//      const AlignmentAlgorithmLine2DLinear::PointEstimateType& li= v1->estimate();
//      const AlignmentAlgorithmLine2DLinear::PointEstimateType& lj= v2->estimate();
			Vector2d l1 = v1->estimate(); //theta, rho
			Vector2d l2 = v2->estimate();
			
			Vector3d li(cos(l1[0]), sin(l1[0]), l1[1]);
			Vector3d lj(cos(l2[0]), sin(l2[0]), l2[1]);
// 			Vector2d Rn = transform.linear()*li.head<2>();
			
      A.block<1,2>(0,0)=lj.head<2>().transpose();
      A.block<1,2>(1,2)=lj.head<2>().transpose();
//       A.block<1,2>(2,4)=Rn.transpose();
//       A.block<1,1>(2,6)=lj.tail<1>().transpose();
      Vector2d ek = li.head<2>();
      ek -= A*x;
// 			J.setZero();
// 			J.block<1,2>(0,0)=lj.head<2>().transpose();
//      J.block<1,2>(1,2)=lj.head<2>().transpose();
//      J.block<2,2>(2,4)=  transform.linear() * skew_2d(lj.head<2>());
			// TODO 3 linea: variazione di rho:
//      J.block<1,1>(3,6)=lj.tail<1>().transpose();
// 			Vector4d ek;
// 			ek.setOnes();
// 			ek.head<3>() = _ek;
// 			Vector3d ek = _ek;
      H+=A.transpose()*Omega.block<2,2>(0,0)*A;
      b+=A.transpose()*Omega.block<2,2>(0,0)*ek;
			err+= ek.transpose()*Omega.block<2,2>(0,0)*ek;
    }
//     LDLT<Matrix7d> ldlt(H);
//     if (ldlt.isNegative())
//       return false;
//     x=ldlt.solve(b); // using a LDLT factorizationldlt;
//     
//     Matrix3d _X = transform.matrix()+vector2homogeneous_2d(x);
//     
//     // recondition the rotation 
//     JacobiSVD<Matrix2d> svd(_X.block<2,2>(0,0), Eigen::ComputeThinU | Eigen::ComputeThinV);
//     if (svd.singularValues()(0)<.5)
//       return false;
//     Matrix2d R = svd.matrixU()*svd.matrixV().transpose();
//     Isometry2d X = Isometry2d::Identity();
//     X.linear() = R;
//     X.translation() = _X.block<2,1>(0,2);
// 
//     Matrix2d H2 = Matrix2d::Zero();
//     Vector2d b2 = Vector2d::Zero();
// 		
//     for (size_t i=0; i<indices.size(); i++){
//       const Correspondence& c = correspondences[indices[i]];
// 			const EdgeLine2D* edge = static_cast<const EdgeLine2D*>(c.edge());
//       const VertexLine2D* v1 = static_cast<const VertexLine2D*>(edge->vertex(0));
//       const VertexLine2D* v2 = static_cast<const VertexLine2D*>(edge->vertex(1));
// //       const AlignmentAlgorithmLine2DLinear::PointEstimateType& li= v1->estimate();
// //       const AlignmentAlgorithmLine2DLinear::PointEstimateType& lj= v2->estimate();
// 			Vector2d l1 = v1->estimate(); //theta, rho
// 			Vector2d l2 = v2->estimate();
// 			
// 			Vector3d li(cos(l1[0]), sin(l1[0]), l1[1]);
// 			Vector3d lj(cos(l2[0]), sin(l2[0]), l2[1]);
// TODO form here
			//ek: parte n:    li.n - R*lj.n
			//		parte rho:  li.rho -lj.rho-R*lj.n*t
// 			Matrix2d J2 = R * skew_2d(lj.head<2>());
// 			Vector3d ek;
// 			ek.block<2,1>(0,0) = li.head<2>() - R * lj.head<2>();
// 			ek[2] = li[2] - lj[2] -J2*X.translation();//non può essere, sottraggo un vettore ad un numero ??
//       Matrix2d A2=skew_2d(Vector2d(R*lj.head<2>()));
//       Vector3d ek = li.w()-R*lj.head<2>()-A2*X.translation();
//       H2+=A2.transpose()*A2;
//       b2+=A2.transpose()*ek;
//     }
//     Vector3d dt;
//     dt = H2.ldlt().solve(b2);
//     X.translation()+=dt;
//     transform = X;
//     cerr << "transform: " << endl;
//     cerr << g2o::internal::toVectorMQT(transform) << endl;;
    return true;
  }


  RansacLine2DLinear::RansacLine2DLinear():  GeneralizedRansac<AlignmentAlgorithmLine2DLinear>(2){
  }

}
