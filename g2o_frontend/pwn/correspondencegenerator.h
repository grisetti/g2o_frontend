#ifndef _CORRESPONDENCEGENERATOR_H_
#define _CORRESPONDENCEGENERATOR_H_

#include "depthimage.h"
#include "homogeneousvector4f.h"
#include "homogeneouspoint3fomega.h"
#include "homogeneouspoint3fstats.h"
#include "pointprojector.h"

struct Correspondence{
  Correspondence(int referenceIndex_ = -1, int currentIndex_ = -1) {
    referenceIndex = referenceIndex_;
    currentIndex = currentIndex_;
  }
  int referenceIndex, currentIndex;
};

typedef std::vector<Correspondence> CorrespondenceVector;

class CorrespondenceGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  CorrespondenceGenerator() {
  _inlierDistanceThreshold = 3.0f;  
  _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
  _inlierNormalAngularThreshold = cos(M_PI/6);
  _flatCurvatureThreshold = 0.02f;
  _inlierCurvatureRatioThreshold = 0.2f;
  _numCorrespondences = 0;
}

  inline int numCorrespondences() { return _numCorrespondences; }
  inline float squaredThreshold() { return _squaredThreshold; }
  inline float inlierDistanceThreshold() { return _inlierDistanceThreshold; }
  inline void setInlierDistanceThreshold(float inlierDistanceThreshold_) {
    _inlierDistanceThreshold = inlierDistanceThreshold_;
    _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
  }
  inline float flatCurvatureThreshold() { return _flatCurvatureThreshold; }
  inline void setFlatCurvatureThreshold(float flatCurvatureThreshold_) { _flatCurvatureThreshold = flatCurvatureThreshold_; }
  inline float inlierCurvatureRatioThreshold() { return _inlierCurvatureRatioThreshold; }
  inline void setInlierCurvatureRatioThreshold(float inlierCurvatureRatioThreshold_) { _inlierCurvatureRatioThreshold = inlierCurvatureRatioThreshold_; }
  inline float inlierNormalAngularThreshold() { return _inlierNormalAngularThreshold; }
  inline void setInlierNormalAngularThreshold(float inlierNormalAngularThreshold_) { _inlierNormalAngularThreshold = inlierNormalAngularThreshold_; }
  
  void compute(CorrespondenceVector &correspondences,
	       const HomogeneousPoint3fVector &referencePoints, const HomogeneousPoint3fVector &currentPoints,
	       const HomogeneousNormal3fVector &referenceNormals, const HomogeneousNormal3fVector &currentNormals,
	       Eigen::MatrixXi &referenceIndexImage, const Eigen::MatrixXi &currentIndexImage,
	       const HomogeneousPoint3fStatsVector &referenceStats, const HomogeneousPoint3fStatsVector &currentStats,
	       Eigen::Isometry3f &T);

 protected:
  float _squaredThreshold;
  float _inlierNormalAngularThreshold;
  float _flatCurvatureThreshold;
  float _inlierCurvatureRatioThreshold;
  float _inlierDistanceThreshold;
  int _numCorrespondences;
};

#endif
