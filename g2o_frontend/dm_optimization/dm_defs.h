#ifndef DM_DEFS_H
#define DM_DEFS_H

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

typedef std::vector<Vector6f,Eigen::aligned_allocator<Vector6f> > Vector6fVector;
typedef std::vector<Matrix6f,Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

typedef Eigen::Matrix<Vector6f*, Eigen::Dynamic, Eigen::Dynamic> Vector6fPtrMatrix;
typedef Eigen::Matrix<Matrix6f*, Eigen::Dynamic, Eigen::Dynamic> Matrix6fPtrMatrix;

struct Correspondence{
  Correspondence (const Vector6f* p1_, const Vector6f* p2_): p1(p1_), p2(p2_) {}
  const Vector6f* p1, *p2;
};

typedef std::vector<Correspondence> CorrespondenceVector;

typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> MatrixXus;
typedef Eigen::DiagonalMatrix<float, 3, 3> Diagonal3f;

#endif
