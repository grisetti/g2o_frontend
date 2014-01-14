#pragma once

#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/pwn_core/cloud.h"
#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/pwn_boss/depthimageconverter.h"
#include "g2o_frontend/boss_map/sensor_data_node.h"
#include "g2o_frontend/boss_map_building/cache.h"

namespace pwn_tracker {
  using namespace cache_ns;
  using namespace pwn;
  using namespace boss_map;

  class PwnCloudCache;

  struct CloudWithImageSize: public pwn::Cloud{
    int imageRows, imageCols;
  };

  class PwnCloudCacheEntry: public CacheEntry<SyncSensorDataNode, CloudWithImageSize>{
  public:
    PwnCloudCacheEntry(PwnCloudCache* cache, SyncSensorDataNode* k, CloudWithImageSize* d=0);
  protected:
    virtual DataType* fetch(KeyType* k);
    PwnCloudCache* _pwnCache;
  };

  class PwnCloudCache: public cache_ns::Cache<PwnCloudCacheEntry>, public boss::Identifiable{
  public:
    PwnCloudCache(DepthImageConverter* converter_ = 0,
		  RobotConfiguration* robotConfiguration_ = 0,
		  const std::string& topic_ = "",
		  int scale_ = 4, int minSlots_ = 100, int _maxSlots_=150, 
		  int id = -1, boss::IdContext* context = 0);

    inline DepthImageConverter* converter() {return _converter;}
    inline void setConverter(DepthImageConverter* converter_) {_converter = converter_;}

    inline const std::string& topic() const { return _topic; }
    inline void setTopic(const std::string& topic_) { _topic = topic_; }

    inline int scale() const {return _scale;}
    inline void setScale(int scale_)  {_scale=scale_;}

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserializeComplete();


    CloudWithImageSize* loadCloud(SyncSensorDataNode* trackerNode);
    double cumTime;
    int numCalls;
    RobotConfiguration* _robotConfiguration;
  protected:
    virtual Cache<PwnCloudCacheEntry>::EntryType* makeEntry(KeyType* k, DataType* d);
    DepthImageConverter* _converter;
    int _scale;
    std::string _topic;
    pwn_boss::DepthImageConverter* _tempConverter;
  };


  class PwnCloudCacheHandler: public MapManagerActionHandler {
  public:
    PwnCloudCacheHandler(MapManager* _manager=0, PwnCloudCache* cache=0, int id=-1, boss::IdContext* context=0);
    inline PwnCloudCache* cache() {return _cache;}
    virtual ~PwnCloudCacheHandler();
    void init();

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserializeComplete();

    virtual void nodeAdded(MapNode* n);
    virtual void nodeRemoved(MapNode* n);
    virtual void relationAdded(MapNodeRelation* _r);
    virtual void relationRemoved(MapNodeRelation* r);
  protected:
    PwnCloudCache* _cache;
  };

  
}
