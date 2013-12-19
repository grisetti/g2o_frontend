#pragma once
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map_building/map_closer.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include <set>
#include <list>

namespace pwn_tracker {

  using namespace std;
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;
  using namespace boss_map_building;
  using namespace pwn_tracker;
  
  struct PwnCloserRelation: public PwnTrackerRelation, ClosureInfo {
    PwnCloserRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
  };

  class PwnCloser: public boss_map_building::MapCloser {
  public:

    PwnCloser(PwnTracker* tracker);

    inline boss_map::PoseAcceptanceCriterion* criterion() {return _criterion;}
    void setCriterion(boss_map::PoseAcceptanceCriterion* criterion_) { _criterion= criterion_;}

    inline PwnCloudCache* cache() {return _cache;}

    virtual void processPartition(std::list<MapNodeBinaryRelation*>& newRelations, std::set<MapNode*> & otherPartition, MapNode* current_);
    PwnCloserRelation* registerNodes(SensingFrameNode* keyNode, SensingFrameNode* otherNode, const Eigen::Isometry3d& initialGuess);
  protected:
    PwnCloudCache* _cache;
    PwnTracker* _tracker;
    PwnMatcherBase* _matcher;
    RobotConfiguration* _robotConfiguration;
    boss_map::PoseAcceptanceCriterion* _criterion;
    int _frameMinNonZeroThreshold;
    int _frameMaxOutliersThreshold;
    int _frameMinInliersThreshold;
  };

  class PwnCloserActiveRelationSelector: public MapRelationSelector {
  public:
    PwnCloserActiveRelationSelector(boss_map::MapManager* manager);
    virtual bool accept(MapNodeRelation* r);
  };

// closure actions
/*
struct NewFrameCloserAdder: public PwnTracker::NewFrameAction {
  NewFrameCloserAdder(PwnCloser* closer, PwnTracker* tracker);
  void compute (SensingFrameNode* frame);
  PwnCloser* _closer;
};


  struct CloserRelationAdder: public PwnTracker::NewRelationAction {
    CloserRelationAdder(std::list<Serializable*>& objects_,
			PwnCloser* closer, 
			MapG2OReflector* optimizer_, 
			PwnTracker* tracker);
    void compute (PwnTrackerRelation* relation);
  protected:
    PwnCloser* _closer;
    MapG2OReflector* _optimizer;
    std::list<Serializable*>& _objects;
  };
}
*/
}
