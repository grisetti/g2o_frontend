#pragma once

#include "stats.h"
#include "informationmatrix.h"

namespace pwn {

  class InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    InformationMatrixCalculator() {
      _flatInformationMatrix.setZero();
      _nonFlatInformationMatrix.setZero();
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _curvatureThreshold = 0.02f;
    }
    virtual ~InformationMatrixCalculator() {}

    inline InformationMatrix flatInformationMatrix() const { return _flatInformationMatrix; }
    inline void setFlatInformationMatrix(const InformationMatrix flatInformationMatrix_) { _flatInformationMatrix = flatInformationMatrix_; }

    inline InformationMatrix nonFlatInformationMatrix() const { return _nonFlatInformationMatrix; }
    inline void setNonFlatInformationMatrix(const InformationMatrix nonFlatInformationMatrix_) { _nonFlatInformationMatrix = nonFlatInformationMatrix_; }

    inline float curvatureThreshold() const { return _curvatureThreshold; }
    inline void setCurvatureThreshold(const float curvatureThreshold_) { _curvatureThreshold = curvatureThreshold_; }

    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &stats,
			 const NormalVector &imageNormals) = 0;
  
  protected:
    float _curvatureThreshold;
    InformationMatrix _flatInformationMatrix;
    InformationMatrix _nonFlatInformationMatrix;
  };

  class PointInformationMatrixCalculator : virtual public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointInformationMatrixCalculator() : InformationMatrixCalculator() {
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1000.0f, 1.0f, 1.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _curvatureThreshold = 0.02f;
    }
    virtual ~PointInformationMatrixCalculator() {}

    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &statsVector,
			 const NormalVector &imageNormals);
  };

  class NormalInformationMatrixCalculator : virtual public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    NormalInformationMatrixCalculator() : InformationMatrixCalculator() {
      _flatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(100.0f, 100.0f, 100.0f));
      _nonFlatInformationMatrix.diagonal() = Normal(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
      _curvatureThreshold = 0.02f;
    }
    virtual ~NormalInformationMatrixCalculator() {}

    virtual void compute(InformationMatrixVector &informationMatrix,
			 const StatsVector &statsVector,
			 const NormalVector &imageNormals);
  };

}
