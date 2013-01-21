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
    _used = false;
  }
  inline Eigen::Matrix3f omegaAcc() const { return _omegaAcc; }
  inline Eigen::Vector3f pointsAcc() const { return _pointsAcc; }
  inline bool used() const { return _used; }
  Eigen::Matrix3f _omegaAcc;
  Eigen::Vector3f _pointsAcc;
  bool _used;
};

typedef Eigen::Matrix<CovarianceAccumulator, Eigen::Dynamic, Eigen::Dynamic> CovarianceAccumulatorMatrix;

class PointWithNormalMerger {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PointWithNormalMerger();
  inline void setAlpha(float alpha_) { _alpha = alpha_; }
  inline void setBaseLine(float baseLine_) { _baseLine = baseLine_; }
  inline void setCameraMatrix(Eigen::Matrix3f cameraMatrix_) { _cameraMatrix = cameraMatrix_; }
  inline float alpha() const { return _alpha; }
  inline float baseLine() const { return _baseLine; }
  inline Eigen::Matrix3f cameraMatrix() { return _cameraMatrix; }
  inline Eigen::MatrixXi* indexImage() { return &_indexImage; }
  inline Eigen::MatrixXf* depthImage() { return &_depthImage; }
  inline PointWithNormalVector* points() { return &_points; }
  inline PointWithNormalVector* mergedPoints() { return &_mergedPoints; }
  inline int size() { return _points.size(); }
  void clearAll();
  void addCloud(Eigen::Isometry3f t, const PointWithNormalVector points_);
  void computeAccumulator();
  void extractMergedCloud();

 protected:
  float _alpha;
  float _baseLine;
  float _scale;
  Eigen::Matrix3f _cameraMatrix;
  Eigen::MatrixXi _indexImage;
  Eigen::MatrixXf _depthImage;
  PointWithNormalVector _points;
  PointWithNormalVector _mergedPoints;
  CovarianceAccumulatorMatrix _covariances;
};

#endif
