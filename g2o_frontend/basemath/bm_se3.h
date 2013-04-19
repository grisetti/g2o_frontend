#ifndef _BM_SE3_H_
#define _BM_SE3_H_

#include "bm_defs.h"

inline Eigen::Matrix3f quat2mat(const Eigen::Vector3f& q)
{
  const float& qx = q.x();
  const float& qy = q.y();
  const float& qz = q.z();
  float qw = sqrt(1.f - q.squaredNorm());
  Eigen::Matrix3f R;
  R << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy - qw*qz) , 2*(qx*qz + qw*qy),
    2*(qx*qy + qz*qw) , qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz - qx*qw),
    2*(qx*qz - qy*qw) , 2*(qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz;

    return R;
}


inline Eigen::Vector3f mat2quat(const Eigen::Matrix3f& R)
{
  Eigen::Quaternionf q(R); 
  q.normalize();
  Eigen::Vector3f rq;
  rq << q.x(), q.y(), q.z();
  if (q.w()<0){
    rq = -rq;
  }
  return rq;
}

inline Eigen::Isometry3f v2t(const Vector6f& x)
{
    Eigen::Isometry3f X;
    X.linear() = quat2mat(x.tail<3>());
    X.translation() = x.head<3>();

    return X;
}

inline Vector6f t2v(const Eigen::Isometry3f& X)
{
    Vector6f v;
    v.head<3>() = X.translation();
    v.tail<3>() = mat2quat(X.linear());

    return v;
}

inline Eigen::Matrix3f skew(const Eigen::Vector3f& v)
{
    const float& tx = 2*v.x();
    const float& ty = 2*v.y();
    const float& tz = 2*v.z();
    Eigen::Matrix3f S;
    S <<  0,    tz, -ty,
         -tz,   0,   tx,
          ty,  -tx,  0;
    return S;
}

inline Eigen::Matrix4f skew4f(const Eigen::Vector3f& v)
{
    const float& tx = v.x();
    const float& ty = v.y();
    const float& tz = v.z();
    Eigen::Matrix4f S;
    S << 0, (2*tz), (-2*ty), 0,
      (-2*tz), 0, (2*tx), 0,				
      (2*ty),  (-2*tx), 0, 0,
      0, 0, 0, 0;
    return S;
}

inline Vector12f homogeneous2vector(const Eigen::Matrix4f& transform){
  Vector12f x;
  x.block<3,1>(0,0)=transform.block<1,3>(0,0).transpose();
  x.block<3,1>(3,0)=transform.block<1,3>(1,0).transpose();
  x.block<3,1>(6,0)=transform.block<1,3>(2,0).transpose();
  x.block<3,1>(9,0)=transform.block<3,1>(0,3);
  return x;
}
  
inline Eigen::Matrix4f vector2homogeneous(const Vector12f x){
  Eigen::Isometry3f transform=Eigen::Isometry3f::Identity();
  transform.matrix().block<1,3>(0,0)=x.block<3,1>(0,0).transpose();
  transform.matrix().block<1,3>(1,0)=x.block<3,1>(3,0).transpose();
  transform.matrix().block<1,3>(2,0)=x.block<3,1>(6,0).transpose();
  transform.translation()=x.block<3,1>(9,0);
  return transform.matrix();
}

/**************  double counterparts **************/


inline Eigen::Matrix3d quat2mat(const Eigen::Vector3d& q)
{
  const double& qx = q.x();
  const double& qy = q.y();
  const double& qz = q.z();
  float qw = sqrt(1.f - q.squaredNorm());
  Eigen::Matrix3d R;
  R << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy - qw*qz) , 2*(qx*qz + qw*qy),
    2*(qx*qy + qz*qw) , qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz - qx*qw),
    2*(qx*qz - qy*qw) , 2*(qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz;
    return R;
}

inline Eigen::Vector3d mat2quat(const Eigen::Matrix3d& R)
{
  Eigen::Quaterniond q(R); 
  q.normalize();
  Eigen::Vector3d rq;
  rq << q.x(), q.y(), q.z();
  if (q.w()<0){
    rq = -rq;
  }
  return rq;
}

inline Eigen::Isometry3d v2t(const Vector6d& x)
{
    Eigen::Isometry3d X;
    X.linear() = quat2mat(Eigen::Vector3d(x.tail<3>()));
    X.translation() = x.head<3>();

    return X;
}

inline Vector6d t2v(const Eigen::Isometry3d& X)
{
    Vector6d v;
    v.head<3>() = X.translation();
    v.tail<3>() = mat2quat(Eigen::Matrix3d(X.linear()));

    return v;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    const double& tx = v.x();
    const double& ty = v.y();
    const double& tz = v.z();
    Eigen::Matrix3d S;
    S << 0, (2*tz), (-2*ty),
            (-2*tz), 0, (2*tx),
            (2*ty),  (-2*tx),0;
    return S;
}

inline Eigen::Matrix4d skew4d(const Eigen::Vector3d& v)
{
    const double& tx = v.x();
    const double& ty = v.y();
    const double& tz = v.z();
    Eigen::Matrix4d S;
    S << 0, (2*tz), (-2*ty), 0,
      (-2*tz), 0, (2*tx), 0,				
      (2*ty),  (-2*tx), 0, 0,
      0, 0, 0, 0;
    return S;
}

inline Vector12d homogeneous2vector(const Eigen::Matrix4d& transform){
  Vector12d x;
  x.block<3,1>(0,0)=transform.block<1,3>(0,0).transpose();
  x.block<3,1>(3,0)=transform.block<1,3>(1,0).transpose();
  x.block<3,1>(6,0)=transform.block<1,3>(2,0).transpose();
  x.block<3,1>(9,0)=transform.block<3,1>(0,3);
  return x;
}
  
inline Eigen::Matrix4d vector2homogeneous(const Vector12d x){
  Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
  transform.matrix().block<1,3>(0,0)=x.block<3,1>(0,0).transpose();
  transform.matrix().block<1,3>(1,0)=x.block<3,1>(3,0).transpose();
  transform.matrix().block<1,3>(2,0)=x.block<3,1>(6,0).transpose();
  transform.translation()=x.block<3,1>(9,0);
  return transform.matrix();
}

#endif
