#pragma once
#include "base_tracker.h"
#include "pwn_matcher_base.h"
#include "pwn_cloud_cache.h"

namespace pwn_tracker{
  using namespace boss_map_building;
  using namespace boss_map;

  class PwnTrackerRelation: public MapNodeBinaryRelation {
  public:
    PwnTrackerRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);

    virtual ~PwnTrackerRelation();

    void fromResult(PwnMatcherBase::MatcherResult& result);

    int cloud_inliers;
    int image_nonZeros;
    int image_outliers;
    int image_inliers;
    float image_reprojectionDistance;
    
  };

  class PwnTracker: public BaseTracker {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PwnTracker(PwnMatcherBase* matcher_,
	       PwnCloudCache* cache_,
	       MapManager* manager_,
	       RobotConfiguration* configuration_);

    int scale() const;
    void setScale (int scale_);

    void setImageSize(int imageRows_, int imageCols_);
    inline int imageRows() const {return _imageRows;}
    inline int imageCols() const {return _imageCols;}

    inline float newFrameCloudInliersFraction() const {return _newFrameCloudInliersFraction; }
    inline void  setNewFrameCloudInliersFraction(float v) { _newFrameCloudInliersFraction = v; }
    inline const std::string& topic() const { return _topic; }
    inline void setTopic(const std::string& topic_) { _topic = topic_; _cache-> setTopic(topic_);}

    inline PwnCloudCache* cache() {return _cache;}
    inline PwnMatcherBase* matcher() {return _matcher;}

    //virtual void init();

    virtual bool shouldChangeKeyframe(MapNodeBinaryRelation* r);
    virtual MapNodeBinaryRelation* registerNodes(MapNode* keyNode, MapNode* otherNode);

    virtual ~PwnTracker();
  protected:
    std::string _topic;
    PwnCloudCache* _cache;
    PwnMatcherBase* _matcher;
    float _newFrameCloudInliersFraction;
    int _imageRows, _imageCols, _imageSize;
    int _scaledImageRows, _scaledImageCols, _scaledImageSize;
  };

}
