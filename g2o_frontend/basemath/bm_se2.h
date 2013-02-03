#ifndef _BM_SE2_H_
#define _BM_SE2_H_

#include "bm_defs.h"

inline Eigen::Matrix2f angle2mat_2d(const Element1f& angle)
{
  Eigen::Matrix2f R;
	float theta = angle[0];
  R << cos(theta), -sin(theta) ,
    sin(theta), cos(theta);

  return R;
}


// inline Eigen::Vector3f mat2quat_2d(const Eigen::Matrix3f& R)
// {
//   Eigen::Quaternionf q(R); 
//   q.normalize();
//   Eigen::Vector3f rq;
//   rq << q.x(), q.y(), q.z();
//   if (q.w()<0){
//     rq = -rq;
//   }
//   return rq;
// }

inline Eigen::Isometry2f v2t_2d(const Eigen::Vector3f& x)
{
    Eigen::Isometry2f X;
    X.linear() = angle2mat_2d(x.tail<1>());
    X.translation() = x.head<2>();

    return X;
}

// inline Vector6f t2v_2d(const Eigen::Isometry3f& X)
// {
//     Vector6f v;
//     v.head<3>() = X.translation();
//     v.tail<3>() = mat2quat_2d(X.linear());
// 
//     return v;
// }

inline Eigen::Matrix3f skew_2d(const Eigen::Vector3f& v)
{
//     const float& tx = v.x();
//     const float& ty = v.y();
    const float& tz = v.z();
    Eigen::Matrix3f S;
     S << 0, (-2*tz), 0,
         (2*tz), 0, 0,
         0, 0,0;
    return S;
}

// inline Vector12f homogeneous2vector(const Eigen::Matrix4f& transform){
//   Vector12f x;
//   x.block<3,1>(0,0)=transform.block<1,3>(0,0).transpose();
//   x.block<3,1>(3,0)=transform.block<1,3>(1,0).transpose();
//   x.block<3,1>(6,0)=transform.block<1,3>(2,0).transpose();
//   x.block<3,1>(9,0)=transform.block<3,1>(0,3);
//   return x;
// }
//   
// inline Eigen::Matrix4f vector2homogeneous(const Vector12f x){
//   Eigen::Isometry3f transform=Eigen::Isometry3f::Identity();
//   transform.matrix().block<1,3>(0,0)=x.block<3,1>(0,0).transpose();
//   transform.matrix().block<1,3>(1,0)=x.block<3,1>(3,0).transpose();
//   transform.matrix().block<1,3>(2,0)=x.block<3,1>(6,0).transpose();
//   transform.translation()=x.block<3,1>(9,0);
//   return transform.matrix();
// }

/**************  double counterparts **************/

inline Eigen::Matrix2d angle2mat_2d(const Element1d& angle)
{
  Eigen::Matrix2d R;
	double theta = angle[0];
  R << cos(theta), -sin(theta) ,
    sin(theta), cos(theta);

  return R;
}

// inline Eigen::Vector3d mat2quat_2d(const Eigen::Matrix3d& R)
// {
//   Eigen::Quaterniond q(R); 
//   q.normalize();
//   Eigen::Vector3d rq;
//   rq << q.x(), q.y(), q.z();
//   if (q.w()<0){
//     rq = -rq;
//   }
//   return rq;
// }

inline Eigen::Isometry2d v2t_2d(const Eigen::Vector3d& x)
{
    Eigen::Isometry2d X;
    X.linear() = angle2mat_2d(Element1d(x.tail<1>()));
    X.translation() = x.head<2>();

    return X;
}

// inline Vector6d t2v_2d(const Eigen::Isometry3d& X)
// {
//     Vector6d v;
//     v.head<3>() = X.translation();
//     v.tail<3>() = mat2quat_2d(Eigen::Matrix3d(X.linear()));
// 
//     return v;
// }
// TODO to be modified for a rotation on z-axis
inline Eigen::Matrix3d skew_2d(const Eigen::Vector3d& v)
{
//     const double& tx = v.x();
//     const double& ty = v.y();
    const double& tz = v.z();
    Eigen::Matrix3d S;
    S << 0, (-2*tz), 0,
         (2*tz), 0, 0,
         0, 0,0;
    return S;
}


// inline Vector12d homogeneous2vector(const Eigen::Matrix4d& transform){
//   Vector12d x;
//   x.block<3,1>(0,0)=transform.block<1,3>(0,0).transpose();
//   x.block<3,1>(3,0)=transform.block<1,3>(1,0).transpose();
//   x.block<3,1>(6,0)=transform.block<1,3>(2,0).transpose();
//   x.block<3,1>(9,0)=transform.block<3,1>(0,3);
//   return x;
// }
//   
// inline Eigen::Matrix4d vector2homogeneous(const Vector12d x){
//   Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
//   transform.matrix().block<1,3>(0,0)=x.block<3,1>(0,0).transpose();
//   transform.matrix().block<1,3>(1,0)=x.block<3,1>(3,0).transpose();
//   transform.matrix().block<1,3>(2,0)=x.block<3,1>(6,0).transpose();
//   transform.translation()=x.block<3,1>(9,0);
//   return transform.matrix();
// }

#endif
