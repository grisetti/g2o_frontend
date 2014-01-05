#pragma once
#include "g2o_frontend/boss_map_building/base_tracker.h"
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

    PwnTracker(PwnMatcherBase* matcher_=0,
	       PwnCloudCache* cache_=0,
	       MapManager* manager_=0,
	       RobotConfiguration* configuration_=0,
	       int id=0, boss::IdContext* context=0);

    //! boss serialization
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    //! boss deserialization
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    //! boss deserialization
    virtual void deserializeComplete();

    int scale() const;
    void setScale (int scale_);

    void setImageSize(int imageRows_, int imageCols_);
    inline int imageRows() const {return _imageRows;}
    inline int imageCols() const {return _imageCols;}

    inline float newFrameCloudInliersFraction() const {return _newFrameCloudInliersFraction; }
    inline void  setNewFrameCloudInliersFraction(float v) { _newFrameCloudInliersFraction = v; }


    inline int cloudMinInliersThreshold() const { return _minCloudInliers; }
    inline void  setCloudMinInliersThreshold( int t)  { _minCloudInliers = t; }

    inline int frameMinNonZeroThreshold() const { return _frameMinNonZeroThreshold; }
    inline void  setFrameMinNonZeroThreshold( int t)  { _frameMinNonZeroThreshold = t; }

    inline int frameMaxOutliersThreshold() const { return _frameMaxOutliersThreshold; }
    inline void  setFrameMaxOutliersThreshold( int t) { _frameMaxOutliersThreshold = t; }

    inline int frameMinInliersThreshold() const { return _frameMinInliersThreshold; }
    inline void  setFrameMinInliersThreshold( int t)  { _frameMinInliersThreshold = t; }
     

    inline bool enabled() const { return _enabled; };
    inline void setEnabled(bool e) { _enabled = e; }

    inline const std::string& topic() const { return _topic; }
    inline void setTopic(const std::string& topic_) { _topic = topic_; if(_cache) _cache-> setTopic(topic_);}

    inline PwnCloudCache* cache() {return _cache;}
    inline PwnMatcherBase* matcher() {return _matcher;}

    //virtual void init();

    virtual bool shouldChangeKeyNode(MapNodeBinaryRelation* r);
    virtual MapNodeBinaryRelation* registerNodes(MapNode* keyNode, 
						 MapNode* otherNode, 
						 const Eigen::Isometry3d& guess = Eigen::Isometry3d::Identity());


    inline void setRobotConfiguration(RobotConfiguration* conf) {_robotConfiguration = conf; _cache->_robotConfiguration = conf;} 
    inline RobotConfiguration* robotConfiguration() const { return _robotConfiguration; }
    virtual ~PwnTracker();
  protected:
    RobotConfiguration* _robotConfiguration;
    std::string _topic;
    PwnCloudCache* _cache;
    PwnMatcherBase* _matcher;
    float _newFrameCloudInliersFraction;
    int _imageRows, _imageCols, _imageSize;
    int _scaledImageRows, _scaledImageCols, _scaledImageSize;
    int _minCloudInliers;
    int _frameMinNonZeroThreshold;
    int _frameMaxOutliersThreshold;
    int _frameMinInliersThreshold;
    bool _enabled;
  };

}
