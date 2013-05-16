#ifndef _OMEGAGENERATOR_H_
#define _OMEGAGENERATOR_H_

#include "homogeneouspoint3fstats.h"
#include "homogeneouspoint3fomega.h"

typedef Eigen::DiagonalMatrix<float, 4> Diagonal4f;

class OmegaGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  OmegaGenerator() {
    _flatOmega.setZero();
    _nonFlatOmega.setZero();
    _flatOmega.diagonal() = HomogeneousNormal3f(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _nonFlatOmega.diagonal() = HomogeneousNormal3f(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 1.0f;
  }

  inline HomogeneousPoint3fOmega flatOmega() const { return _flatOmega; }
  inline HomogeneousPoint3fOmega nonFlatOmega() const { return _nonFlatOmega; }
  inline float curvatureThreshold() const { return _curvatureThreshold; }

  inline void setFlatOmega(const HomogeneousPoint3fOmega flatOmega_) { _flatOmega = flatOmega_; }
  inline void setNonFlatOmega(const HomogeneousPoint3fOmega nonFlatOmega_) { _nonFlatOmega = nonFlatOmega_; }
  inline void setCurvatureThreshold(const float curvatureThreshold_) { _curvatureThreshold = curvatureThreshold_; }

  virtual void compute(HomogeneousPoint3fOmegaVector &omegas, 
		       const HomogeneousPoint3fStatsVector &stats,
		       const HomogeneousNormal3fVector &imageNormals) = 0;
  
 protected:
  HomogeneousPoint3fOmega _flatOmega;
  HomogeneousPoint3fOmega _nonFlatOmega;
  float _curvatureThreshold;
};

class PointOmegaGenerator : OmegaGenerator {
 public:
  PointOmegaGenerator() {
    _flatOmega.diagonal() = HomogeneousNormal3f(Eigen::Vector3f(1000.0f, 1.0f, 1.0f));
    _nonFlatOmega.diagonal() = HomogeneousNormal3f(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 0.02f;
  }

  virtual void compute(HomogeneousPoint3fOmegaVector &omegas, 
		       const HomogeneousPoint3fStatsVector &stats,
		       const HomogeneousNormal3fVector &imageNormals);
};

class NormalOmegaGenerator : OmegaGenerator {
 public:
  NormalOmegaGenerator() {
    _flatOmega.diagonal() = HomogeneousNormal3f(Eigen::Vector3f(100.0f, 100.0f, 100.0f));
    _nonFlatOmega.diagonal() = HomogeneousNormal3f(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    _curvatureThreshold = 0.02f;
  }

  virtual void compute(HomogeneousPoint3fOmegaVector &omegas, 
		       const HomogeneousPoint3fStatsVector &stats,
		       const HomogeneousNormal3fVector &imageNormals);
};

#endif
