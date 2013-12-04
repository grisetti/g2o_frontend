#ifndef _PWN_MERGER_H
#define _PWN_MERGER_H

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "pinholepointprojector.h"
#include "depthimageconverter.h"
#include "frame.h"

namespace pwn {

  class Merger : public boss::Identifiable{
  public:  
    Merger(int id =-1, boss::IdContext* context=0);
    virtual ~Merger();
  
    inline void setDistanceThreshold(float distanceThreshold_) { _distanceThreshold = distanceThreshold_; }
    inline void setNormalThreshold(float normalThreshold_) { _normalThreshold = normalThreshold_; }
    inline void setMaxPointDepth(float maxPointDepth_) { _maxPointDepth = maxPointDepth_; }
    inline void setDepthImageConverter(DepthImageConverter *depthImageConverter_) { _depthImageConverter = depthImageConverter_; }
    inline void setImageSize(int r, int c) {
      _indexImage.resize(r, c);
      _depthImage.resize(r, c);
    }

    inline float distanceThreshold() const { return _distanceThreshold; }
    inline float normalThreshold() const { return _normalThreshold; }
    inline float maxPointDepth() const { return _maxPointDepth; }
    inline DepthImageConverter* depthImageConverter() const { return _depthImageConverter; }
    inline Eigen::Vector2i imageSize() const { return Eigen::Vector2i(_indexImage.rows(), _indexImage.cols()); }  

    void merge(Frame *frame, Eigen::Isometry3f transform = Eigen::Isometry3f::Identity());

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    
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

#endif
