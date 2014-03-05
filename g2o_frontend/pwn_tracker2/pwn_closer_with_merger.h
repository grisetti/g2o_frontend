#pragma once
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map_building/map_closer.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include <set>
#include <list>

#include "merger2.h"


namespace pwn_tracker {

  using namespace std;
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;
  using namespace boss_map_building;
  using namespace pwn_tracker;
  

  class PwnCloserWithMerger: public boss_map_building::MapCloser {
  public:

    PwnCloserWithMerger(PwnMatcherBase* matcher_ = 0,
	      PwnCloudCache* cache_ = 0,
	      MapManager* manager_ = 0,
	      RobotConfiguration* configuration_=0,
	      int id=0, boss::IdContext* context=0);

    
    inline int frameMinNonZeroThreshold() const { return _frameMinNonZeroThreshold; }
    inline void  setFrameMinNonZeroThreshold( int t)  { _frameMinNonZeroThreshold = t; }

    inline int frameMaxOutliersThreshold() const { return _frameMaxOutliersThreshold; }
    inline void  setFrameMaxOutliersThreshold( int t) { _frameMaxOutliersThreshold = t; }


    inline int frameMinInliersThreshold() const { return _frameMinInliersThreshold; }
    inline void  setFrameMinInliersThreshold( int t)  { _frameMinInliersThreshold = t; }

    inline float closureClampingDistance() const {return _closureClampingDistance;}
    inline void setClosureClampingDistance(float closureClampingDiustance_) { _closureClampingDistance = closureClampingDiustance_;}

    inline bool enabled() const { return _enabled; };
    inline void setEnabled(bool e) { _enabled = e; }

    virtual void process(Serializable* s);

    //! boss serialization
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    //! boss deserialization
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

    inline PwnCloudCache* cache() {return _cache;}
    inline void setCache(PwnCloudCache* cache_) {_cache=cache_;}

    inline PwnMatcherBase* matcher() { return _matcher; }
    inline void setMatcher(PwnMatcherBase* matcher_) { _matcher=matcher_; }

    inline void setRobotConfiguration(RobotConfiguration* conf) {_robotConfiguration = conf; if (_cache) _cache->_robotConfiguration = conf;} 
  protected:
    virtual void processPartition(std::list<MapNodeBinaryRelation*>& newRelations, std::set<MapNode*> & otherPartition, MapNode* current_);
	void processCurrentPartition(SyncSensorDataNode* current);
    void mergeNode(DepthImage& out,SyncSensorDataNode* other,Eigen::Isometry3d& T);
						
    
  protected:
    RobotConfiguration* _robotConfiguration;
    PwnCloudCache* _cache;
    //PwnTracker* _tracker;
    PwnMatcherBase* _matcher;
    mutable int _scaledImageSize;
    int _frameMinNonZeroThreshold;
    int _frameMaxOutliersThreshold;
    int _frameMinInliersThreshold;
    bool _enabled;
    float _closureClampingDistance;

    Merger2* _merger;
    std::list<MapNode*> _currentPartitionActive;
    MapNode* _current;
    DepthImage _currentPartitionImage;
    DepthImage _otherPartitionImage;
    std::list<MapNode*> _nodeList;
  };

}
