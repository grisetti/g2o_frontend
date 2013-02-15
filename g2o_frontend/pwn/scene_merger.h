#ifndef _SCENE_MERGER_H_
#define _SCENE_MERGER_H
#include "depthimage.h"
#include "pointwithnormal.h"
#include "pointwithnormalstatsgenerator.h"
#include "pointwithnormalaligner.h"
#include "g2o/stuff/timeutil.h"
#include "scene.h"
#include "pixelmapper.h"



struct SceneMerger{
  std::vector<int> _collapsedIndices;
  Eigen::Matrix3f _cameraMatrix;
  Eigen::MatrixXf _zBuffer;
  Eigen::MatrixXi _indexImage;
  float _distanceThreshold;
  float _normalThreshold;
  float _maxPointDepth;
  PointWithNormalStatistcsGenerator* _normalGenerator;

  SceneMerger();
  inline void setImageSize(int r, int c) {
    _indexImage.resize(r,c);
    _zBuffer.resize(r,c);
  }
  
  inline Eigen::Vector2i imageSize() const {
    return Eigen::Vector2i(_indexImage.rows(), _indexImage.cols());
  }

  inline void setDistanceThreshold(float distanceThreshold_) {
    _distanceThreshold = distanceThreshold_;
  }

  inline float distanceThreshold() const {
    return _distanceThreshold;
  }

  inline void setNormalThreshold(float normalThreshold_) {
    _normalThreshold = normalThreshold_;
  }

  inline float normalThreshold() const {
    return _normalThreshold;
  }
  
  inline void setNormalGenerator(PointWithNormalStatistcsGenerator* normalGenerator_) { _normalGenerator = normalGenerator_;} 

  inline PointWithNormalStatistcsGenerator* normalGenerator() {return _normalGenerator;} 
  
  inline void setCameraMatrix(const Eigen::Matrix3f cameraMatrix_) {_cameraMatrix = cameraMatrix_;}

  void merge(Scene* scene, Eigen::Isometry3f& transform);
};

#endif
