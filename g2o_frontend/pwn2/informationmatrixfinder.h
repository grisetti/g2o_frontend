#ifndef _INFORMATIONMATRIXFINDER_H_
#define _INFORMATIONMATRIXFINDER_H_

#include "pointstats.h"
#include "informationmatrix.h"

namespace pwn {

typedef Eigen::DiagonalMatrix<float, 4> Diagonal4f;

class InformationMatrixFinder {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  InformationMatrixFinder() {
    _flatInformationMatrix.setZero();
    _nonFlatInformationMatrix.setZero();
    _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 1.0f;
  }

  inline InformationMatrix flatInformationMatrix() const { return _flatInformationMatrix; }
  inline InformationMatrix nonFlatInformationMatrix() const { return _nonFlatInformationMatrix; }
  inline float curvatureThreshold() const { return _curvatureThreshold; }

  inline void setFlatInformationMatrix(const InformationMatrix flatInformationMatrix_) { _flatInformationMatrix = flatInformationMatrix_; }
  inline void setNonFlatInformationMatrix(const InformationMatrix nonFlatInformationMatrix_) { _nonFlatInformationMatrix = nonFlatInformationMatrix_; }
  inline void setCurvatureThreshold(const float curvatureThreshold_) { _curvatureThreshold = curvatureThreshold_; }

  virtual void compute(InformationMatrixVector &informationMatrix,
               const PointStatsVector &stats,
               const NormalVector &imageNormals) = 0;
  
 protected:
  InformationMatrix _flatInformationMatrix;
  InformationMatrix _nonFlatInformationMatrix;
  float _curvatureThreshold;
};

class PointInformationMatrixFinder : public InformationMatrixFinder {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PointInformationMatrixFinder() {
    _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 1.0f, 1.0f));
    _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 0.02f;
  }

  virtual void compute(InformationMatrixVector &informationMatrix,
               const PointStatsVector &stats,
               const NormalVector &imageNormals);
};

class NormalInformationMatrixFinder : public InformationMatrixFinder {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  NormalInformationMatrixFinder() {
    _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(100.0f, 100.0f, 100.0f));
    _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 0.02f;
  }

  virtual void compute(InformationMatrixVector &informationMatrix,
               const PointStatsVector &stats,
               const NormalVector &imageNormals);
};

}

#endif
