#ifndef PWN_DEFS_H
#define PWN_DEFS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

struct SVDMatrix3f{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Isometry3f isometry;
  Eigen::Vector3f lambda;
  SVDMatrix3f();
  inline void setZero() { isometry.setIdentity(); lambda.setZero(); }
  inline float curvature() const { 
    if (lambda.squaredNorm()<1e-20) 
      return -1.0f;
    return lambda[0]/(lambda[0]+lambda[1]+lambda[2]);
  }
};

typedef std::vector<SVDMatrix3f, Eigen::aligned_allocator<SVDMatrix3f> > CovarianceSVDVector; 
typedef Eigen::Matrix<SVDMatrix3f*, Eigen::Dynamic, Eigen::Dynamic> CovarianceSVDPtrMatrix;

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

typedef Eigen::Matrix<Vector6f*, Eigen::Dynamic, Eigen::Dynamic> Vector6fPtrMatrix;
typedef Eigen::Matrix<Matrix6f*, Eigen::Dynamic, Eigen::Dynamic> Matrix6fPtrMatrix;

#endif
