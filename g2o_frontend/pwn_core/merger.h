#pragma once

#include "pinholepointprojector.h"
#include "depthimageconverter.h"
#include "cloud.h"

namespace pwn {

  class Merger {
  public:  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Merger();
    virtual ~Merger() {}
  
    inline float distanceThreshold() const { return _distanceThreshold; }
    inline void setDistanceThreshold(float distanceThreshold_) { _distanceThreshold = distanceThreshold_; }

    inline float normalThreshold() const { return _normalThreshold; }	
    inline void setNormalThreshold(float normalThreshold_) { _normalThreshold = normalThreshold_; }

    inline float maxPointDepth() const { return _maxPointDepth; }
    inline void setMaxPointDepth(float maxPointDepth_) { _maxPointDepth = maxPointDepth_; }

    inline DepthImageConverter* depthImageConverter() const { return _depthImageConverter; }
    inline void setDepthImageConverter(DepthImageConverter *depthImageConverter_) { _depthImageConverter = depthImageConverter_; }

    inline Eigen::Vector2i imageSize() const { return Eigen::Vector2i(_indexImage.rows, _indexImage.cols); }  
    inline void setImageSize(int r, int c) {
      _indexImage.create(r, c);
      _depthImage.create(r, c);
    }

    void merge(Cloud *cloud, Eigen::Isometry3f transform = Eigen::Isometry3f::Identity());
    
  protected:
    float _distanceThreshold;
    float _normalThreshold;
    float _maxPointDepth;

    DepthImageConverter *_depthImageConverter;

    DepthImage _depthImage;
    IntImage _indexImage;
    std::vector<int> _collapsedIndices;  
  };

}
