#ifndef _POINTWITHNORMAL_MERGER_
#define _POINTWITHNORMAL_MERGER_

#include <Eigen/Geometry>
#include <vector>
#include "pointwithnormal.h"
#include "pointwithnormalstatsgenerator.h" 

struct CovarianceAccumulator {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CovarianceAccumulator() {
    _omegaAcc = Eigen::Matrix3f::Zero();
    _pointsAcc = Eigen::Vector3f::Zero();
  }
  inline Eigen::Matrix3f omegaAcc() const { return _omegaAcc; }
  inline Eigen::Vector3f pointsAcc() const { return _pointsAcc; }
  Eigen::Matrix3f _omegaAcc;
  Eigen::Vector3f _pointsAcc;
};

typedef std::vector<CovarianceAccumulator> CovarianceAccumulatorVector;

class PointWithNormalMerger {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PointWithNormalMerger();
  inline void setAlpha(float alpha_) { _alpha = alpha_; }
  inline void setBaseLine(float baseLine_) { _baseLine = baseLine_; }
  inline void setCameraMatrix(Eigen::Matrix3f cameraMatrix_) { _cameraMatrix = cameraMatrix_; }
  inline float alpha() const { return _alpha; }
  inline float baseLine() const { return _baseLine; }
  inline Eigen::Matrix3f cameraMatrix() const { return _cameraMatrix; }
  inline const Eigen::MatrixXi* indexImage() const { return &_indexImage; }
  inline const Eigen::MatrixXf* depthImage() const { return &_depthImage; }
  inline const PointWithNormalVector* points() const { return &_points; }
  inline const PointWithNormalVector* mergedPoints() const { return &_mergedPoints; }
  //inline PointWithNormalSVDVector* svds() { return &_covariancesSVDsVector; }
  inline int size() const { return _points.size(); }
  void addCloud(Eigen::Isometry3f t, const PointWithNormalVector points_);
  void computeAccumulator();
  void extractMergedCloud();

 protected:
  float _alpha;
  float _baseLine;
  Eigen::Matrix3f _cameraMatrix;
  Eigen::MatrixXi _indexImage;
  Eigen::MatrixXf _depthImage;
  PointWithNormalVector _points;
  PointWithNormalVector _mergedPoints;
  CovarianceAccumulatorVector _covariancesAccumulator;
  //PointWithNormalSVDVector _covariancesSVDsVector;
};

#endif
