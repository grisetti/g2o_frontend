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

// inline Eigen::Matrix3f skew_2d(const Eigen::Vector2f& v)
// {
// 	float theta = atan2(v[1], v[0]);
// //     const float& tx = v.x();
// //     const float& ty = v.y();
// //     const float& tz = v.z();
//     Eigen::Matrix3f S;
//      S << 0, (2*theta), 0,
//          (-2*theta), 0, 0,
//          0, 0,0;
//     return S;
// }

inline Vector6f homogeneous2vector_2d(const Eigen::Matrix3f& transform){
  Vector6f x;
  x.block<2,1>(0,0)=transform.block<1,2>(0,0).transpose();
  x.block<2,1>(2,0)=transform.block<1,2>(1,0).transpose();
  x.block<2,1>(4,0)=transform.block<2,1>(0,2);
  return x;
}

inline Eigen::Matrix3f vector2homogeneous_2d(const Vector6f x){
  Eigen::Isometry2f transform=Eigen::Isometry2f::Identity();
  transform.matrix().block<1,2>(0,0)=x.block<2,1>(0,0).transpose();
  transform.matrix().block<1,2>(1,0)=x.block<2,1>(2,0).transpose();
  transform.translation()=x.block<2,1>(4,0);
  return transform.matrix();
}

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

inline Eigen::Matrix2d skew_2d(const Eigen::Vector2d& v)
{
	double theta = atan2(v[1], v[0]);
//     const float& tx = v.x();
//     const float& ty = v.y();
//     const float& tz = v.z();
    Eigen::Matrix2d S;
     S << 0, (2*theta),
         (-2*theta), 0;
    return S;
}


inline Vector6d homogeneous2vector_2d(const Eigen::Matrix3d& transform){
  Vector6d x;
  x.block<2,1>(0,0)=transform.block<1,2>(0,0).transpose();
  x.block<2,1>(2,0)=transform.block<1,2>(1,0).transpose();
  x.block<2,1>(4,0)=transform.block<2,1>(0,2);
  return x;
}
  
inline Eigen::Matrix3d vector2homogeneous_2d(const Vector6d x){
  Eigen::Isometry2d transform=Eigen::Isometry2d::Identity();
  transform.matrix().block<1,2>(0,0)=x.block<2,1>(0,0).transpose();
  transform.matrix().block<1,2>(1,0)=x.block<2,1>(2,0).transpose();
  transform.translation()=x.block<2,1>(4,0);
  return transform.matrix();
}

#endif
