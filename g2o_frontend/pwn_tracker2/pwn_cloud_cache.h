#pragma once

#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/pwn_core/cloud.h"
#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/boss_map/sensing_frame_node.h"
#include "g2o_frontend/boss_map_building/cache.h"

namespace pwn_tracker {
  using namespace cache_ns;
  using namespace pwn;
  using namespace boss_map;

  class PwnCloudCache;

  class PwnCloudCacheEntry: public CacheEntry<SensingFrameNode, pwn::Cloud>{
  public:
    PwnCloudCacheEntry(PwnCloudCache* cache, SensingFrameNode* k, pwn::Cloud* d=0);
  protected:
    virtual DataType* fetch(KeyType* k);
    PwnCloudCache* _pwnCache;
  };

  class PwnCloudCache: public cache_ns::Cache<PwnCloudCacheEntry>{
  public:
    PwnCloudCache(DepthImageConverter* converter_, 
		  RobotConfiguration* robotConfiguration_,
		  const std::string& topic_,
		  int scale_, int minSlots_, int _maxSlots_);

    inline DepthImageConverter* converter() {return _converter;}
    inline void setConverter(DepthImageConverter* converter_) {_converter = converter_;}

    inline const std::string& topic() const { return _topic; }
    inline void setTopic(const std::string& topic_) { _topic = topic_; }

    inline int scale() const {return _scale;}
    inline void setScale(int scale_)  {_scale=scale_;}

    pwn::Cloud* loadCloud(SensingFrameNode* trackerNode);
    double cumTime;
    int numCalls;
  protected:
    virtual Cache<PwnCloudCacheEntry>::EntryType* makeEntry(KeyType* k, DataType* d);
    DepthImageConverter* _converter;
    RobotConfiguration* _robotConfiguration;
    int _scale;
    std::string _topic;
  };


  class PwnCloudCacheHandler: public MapManagerActionHandler {
  public:
    PwnCloudCacheHandler(MapManager* _manager, PwnCloudCache* cache);
    inline PwnCloudCache* cache() {return _cache;}
    virtual ~PwnCloudCacheHandler();
    void init();

    virtual void nodeAdded(MapNode* n);
    virtual void nodeRemoved(MapNode* n);
    virtual void relationAdded(MapNodeRelation* _r);
    virtual void relationRemoved(MapNodeRelation* r);
  protected:
    PwnCloudCache* _cache;
  };

  
}
