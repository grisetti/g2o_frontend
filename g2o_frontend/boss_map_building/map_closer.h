#pragma once

#include "g2o_frontend/boss_map_building/map_g2o_reflector.h"
#include "g2o_frontend/boss_map/map_utils.h"

namespace boss_map_building {
  using namespace boss;
  using namespace boss_map;

  struct ClosureInfo {
    ClosureInfo();
    virtual ~ClosureInfo();
    // status
    bool accepted;
    int consensusCumInlier;
    int consensusCumOutlierTimes;
    int consensusTimeChecked;
  };
  
  class MapCloser{
  public:

    MapCloser(MapManager* manager_);
    
    void addFrame(MapNode* f);
    void addRelation(MapNodeBinaryRelation* r);
    void process();
    virtual void processPartition(std::list<MapNodeBinaryRelation*>& newRelations, 
				  std::set<MapNode*> & otherPartition, 
				  MapNode* current_)=0;
    std::list<MapNodeBinaryRelation*>& results() {return _results;}
    std::vector<MapNodeBinaryRelation*> _trackerRelations;
    std::map<int, MapNode*> _trackerFrames;

    std::list<MapNodeBinaryRelation*>& committedRelations() {return _committedRelations;}
    std::list<MapNodeBinaryRelation*>& candidateRelations() {return _candidateRelations;}
    std::vector<std::set<MapNode*> >& partitions() {return _partitions;}
    std::set<MapNode*>* currentPartition() {return _currentPartition;}
    bool _debug;
  protected:
    void validatePartitions(std::set<MapNode*>& other, 
			    std::set<MapNode*>& current);
    std::vector<std::set<MapNode*> > _partitions;
    std::set<MapNode*>* _currentPartition;

    float _consensusInlierTranslationalThreshold;
    float _consensusInlierRotationalThreshold;
    int    _consensusMinTimesCheckedThreshold;
    MapNode* _pendingTrackerFrame, *_lastTrackerFrame;
    boss_map::MapManager* _manager;
    PoseAcceptanceCriterion* _criterion;
    MapRelationSelector* _selector;
    std::list<MapNodeBinaryRelation*> _results;
    std::list<MapNodeBinaryRelation*> _committedRelations;
    std::list<MapNodeBinaryRelation*> _candidateRelations;
  };

}
