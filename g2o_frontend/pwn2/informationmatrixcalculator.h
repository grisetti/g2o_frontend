#ifndef _PWN_INFORMATIONMATRIXCALCULATOR_H_
#define _PWN_INFORMATIONMATRIXCALCULATOR_H_

#include "stats.h"
#include "informationmatrix.h"

namespace pwn {

typedef Eigen::DiagonalMatrix<float, 4> Diagonal4f;

class InformationMatrixCalculator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  InformationMatrixCalculator() {
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
               const StatsVector &stats,
               const NormalVector &imageNormals) = 0;
  
 protected:
  InformationMatrix _flatInformationMatrix;
  InformationMatrix _nonFlatInformationMatrix;
  float _curvatureThreshold;
};

class PointInformationMatrixCalculator : public InformationMatrixCalculator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PointInformationMatrixCalculator() {
    _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 1.0f, 1.0f));
    _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 0.02f;
  }

  virtual void compute(InformationMatrixVector &informationMatrix,
		       const StatsVector &statsVector,
		       const NormalVector &imageNormals);
};

class NormalInformationMatrixCalculator : public InformationMatrixCalculator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  NormalInformationMatrixCalculator() {
    _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(100.0f, 100.0f, 100.0f));
    _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 0.02f;
  }

  virtual void compute(InformationMatrixVector &informationMatrix,
		       const StatsVector &statsVector,
		       const NormalVector &imageNormals);
};

}

#endif
