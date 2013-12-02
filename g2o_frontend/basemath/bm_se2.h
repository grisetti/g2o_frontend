#ifndef _BM_SE2_H_
#define _BM_SE2_H_

#include "bm_defs.h"
#include <g2o/types/slam2d/se2.h>
#include <iostream>

inline Eigen::Matrix2f angle2mat_2f(float theta)
{
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta) ,
    sin(theta), cos(theta);

  return R;
}

inline float mat2angle_2f(const Eigen::Matrix2f& R)
{
  return atan2(R(1,0),R(0,0));
}

inline Eigen::Isometry2f v2t_2f(const Eigen::Vector3f& x)
{
    Eigen::Isometry2f X = Eigen::Isometry2f::Identity();
    X.linear() = angle2mat_2f(x[2]);
    X.translation() = x.head<2>();

    return X;
}

inline Eigen::Vector3f t2v_2f(const Eigen::Isometry2f& X)
{
    Eigen::Vector3f v;
    v.head<2>() = X.translation();
    v[2] = mat2angle_2f(X.linear());

    return v;
}

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

inline Vector6f homogeneous2vector_2f(const Eigen::Matrix3f& transform){
  Vector6f x;
  x.block<2,1>(0,0)=transform.block<1,2>(0,0).transpose();
  x.block<2,1>(2,0)=transform.block<1,2>(1,0).transpose();
  x.block<2,1>(4,0)=transform.block<2,1>(0,2);
  return x;
}

inline Eigen::Matrix3f vector2homogeneous_2f(const Vector6f x){
  Eigen::Isometry2f transform=Eigen::Isometry2f::Identity();
  transform.matrix().block<1,2>(0,0)=x.block<2,1>(0,0).transpose();
  transform.matrix().block<1,2>(1,0)=x.block<2,1>(2,0).transpose();
  transform.translation()=x.block<2,1>(4,0);
  return transform.matrix();
}

/**************  double counterparts **************/

inline Eigen::Matrix2d angle2mat_2d(double theta)
{
  Eigen::Matrix2d R;
  R << cos(theta), -sin(theta) ,
    sin(theta), cos(theta);

  return R;
}

inline double mat2angle_2d(const Eigen::Matrix2d& R)
{
  return (double)atan2(R(1,0),R(0,0));
}

inline Eigen::Isometry2d v2t_2d(const Eigen::Vector3d& x)
{
    Eigen::Isometry2d X = Eigen::Isometry2d::Identity();
    X.linear() = angle2mat_2d(x[2]);
    X.translation() = x.head<2>();

    return X;
}

inline Eigen::Vector3d t2v_2d(const Eigen::Isometry2d& X)
{
    Eigen::Vector3d v;
    v.head<2>() = X.translation();
    v[2] = mat2angle_2d(X.linear());
    return v;
}

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

inline g2o::SE2 iso3toSE_2d(const Eigen::Isometry3d& iso){
  Eigen::AngleAxisd aa(iso.linear());
  float angle = aa.angle();
  if (aa.axis().z()<0){
    angle=-angle;
  }
  return g2o::SE2(iso.translation().x(), iso.translation().y(), angle);
}

#endif
