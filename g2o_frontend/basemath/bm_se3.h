#ifndef _BM_SE3_H_
#define _BM_SE3_H_

#include "bm_defs.h"
#include <iostream>

template <typename OtherDerived>
inline Eigen::Matrix<typename OtherDerived::Scalar,3,3> quat2mat(const Eigen::MatrixBase< OtherDerived >& q)
{
  const typename OtherDerived::Scalar& qx = q.x();
  const typename OtherDerived::Scalar& qy = q.y();
  const typename OtherDerived::Scalar& qz = q.z();
  typename  OtherDerived::Scalar qw = sqrt(1.f - q.squaredNorm());
  Eigen::Matrix<typename  OtherDerived::Scalar,3,3> R;
  R << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy - qw*qz) , 2*(qx*qz + qw*qy),
    2*(qx*qy + qz*qw) , qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz - qx*qw),
    2*(qx*qz - qy*qw) , 2*(qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz;

    return R;
}

template <typename OtherDerived>
inline Eigen::Matrix<typename OtherDerived::Scalar,3,1> mat2quat(const Eigen::MatrixBase< OtherDerived > & R)
{
  Eigen::Quaternion<typename OtherDerived::Scalar> q(R); 
  q.normalize();
  Eigen::Matrix<typename OtherDerived::Scalar,3,1> rq;
  rq << q.x(), q.y(), q.z();
  if (q.w()<0){
    rq = -rq;
  }
  return rq;
}

template <typename OtherDerived>
inline Eigen::Transform<typename OtherDerived::Scalar, 3, Eigen::Isometry> v2t(const Eigen::MatrixBase< OtherDerived >& x)
{
  Eigen::Transform<typename OtherDerived::Scalar, 3, Eigen:: Isometry> X;
  X.linear() = quat2mat(x.tail(3));
  X.translation() = x.head(3);
  return X;
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 6, 1> t2v(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& X)
{
    Eigen::Matrix<Scalar, 6, 1> v;
    v.head(3) = X.translation();
    v.tail(3) = mat2quat(X.linear());

    return v;
}

template <typename OtherDerived>
inline Eigen::Matrix<typename OtherDerived::Scalar,3,3> skew(const Eigen::MatrixBase< OtherDerived >& v)
{
    const typename OtherDerived::Scalar& tx = v.x();
    const typename OtherDerived::Scalar& ty = v.y();
    const typename OtherDerived::Scalar& tz = v.z();
    Eigen::Matrix<typename OtherDerived::Scalar,3,3> S;
    S << 0, (2*tz), (-2*ty),
            (-2*tz), 0, (2*tx),
            (2*ty),  (-2*tx),0;
    return S;
}

template <typename  OtherDerived>
inline Eigen::Matrix<typename OtherDerived::Scalar, 12,1> homogeneous2vector(const Eigen::MatrixBase< OtherDerived >& transform){
  Eigen::Matrix<typename OtherDerived::Scalar, 12, 1> x;
  x.block(3,1,0,0)=transform.block(1,3,0,0).transpose();
  x.block(3,1,3,0)=transform.block(1,3,1,0).transpose();
  x.block(3,1,6,0)=transform.block(1,3,2,0).transpose();
  x.block(3,1,9,0)=transform.block(3,1,0,3);
  std::cerr << x.transpose() << std::endl;
  return x;
}
  
template <typename  OtherDerived>
inline Eigen::Matrix<typename OtherDerived::Scalar,4,4> vector2homogeneous(const Eigen::MatrixBase< OtherDerived >& x){
  Eigen::Transform<typename OtherDerived::Scalar,3,Eigen::Isometry > transform;
  transform.linear().block(1,3,0,0)=x.block(3,1,0,0).transpose();
  transform.linear().block(1,3,1,0)=x.block(3,1,3,0).transpose();
  transform.linear().block(1,3,2,0)=x.block(3,1,6,0).transpose();
  transform.translation()=x.block(3,1,9,0);
  return transform.matrix();
}

#endif
