#ifndef _MERGER_H
#define _MERGER_H

#include "pinholepointprojector.h"
#include "depthimageconverter.h"
#include "frame.h"

namespace pwn {

class Merger {
 public:  
  Merger();
  ~Merger() {}
  
  inline void setDistanceThreshold(float distanceThreshold_) { _distanceThreshold = distanceThreshold_; }
  inline void setNormalThreshold(float normalThreshold_) { _normalThreshold = normalThreshold_; }
  inline void setMaxPointDepth(float maxPointDepth_) { _maxPointDepth = maxPointDepth_; }
  inline void setPointProjector(PinholePointProjector *pointProjector_) { _pointProjector = pointProjector_; }
  inline void setDepthImageConverter(DepthImageConverter *depthImageConverter_) { _depthImageConverter = depthImageConverter_; }
  inline void setImageSize(int r, int c) {
    _indexImage.resize(r, c);
    _depthImage.resize(r, c);
  }

  inline float distanceThreshold() const { return _distanceThreshold; }
  inline float normalThreshold() const { return _normalThreshold; }
  inline float maxPointDepth() const { return _maxPointDepth; }
  inline PinholePointProjector* pointProjector() const { return _pointProjector; }
  inline DepthImageConverter* depthImageConverter() const { return _depthImageConverter; }
  inline Eigen::Vector2i imageSize() const { return Eigen::Vector2i(_indexImage.rows(), _indexImage.cols()); }  

  void merge(Frame *frame, Eigen::Isometry3f &transform);
 
 protected:
  float _distanceThreshold;
  float _normalThreshold;
  float _maxPointDepth;

  PinholePointProjector *_pointProjector;
  DepthImageConverter *_depthImageConverter;

  DepthImage _depthImage;
  Eigen::MatrixXi _indexImage;
  std::vector<int> _collapsedIndices;  
};


}

#endif
