#pragma once

#include "g2o_frontend/boss_map_building/map_g2o_reflector.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include "g2o_frontend/boss_map/stream_processor.h"

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
  
  class ClosureFoundMessage: public Serializable {
  public:
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    std::list<MapNodeBinaryRelation*> closureRelations;
  };

  class ClosureScannedMessage: public Serializable {
  public:
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    std::vector<std::set<MapNode*> > partitions;
    int currentPartitionIndex;
  };

  class MapCloser: public boss_map::StreamProcessor {
  public:

    MapCloser(MapManager* manager_=0, int id=-1, boss::IdContext* context=0);
    
    void addKeyNode(MapNode* f);
    void addRelation(MapNodeRelation* r);
    void process();
    virtual void processPartition(std::list<MapNodeBinaryRelation*>& newRelations, 
				  std::set<MapNode*> & otherPartition, 
				  MapNode* current_)=0;
    std::list<MapNodeBinaryRelation*>& results() {return _results;}

    inline boss_map::PoseAcceptanceCriterion* criterion() {return _criterion;}
    inline void setCriterion(boss_map::PoseAcceptanceCriterion* criterion_) { 
      _criterion= criterion_;
      if (_criterion)
	_criterion->setManager(_manager);
    }

    inline boss_map::MapRelationSelector* selector() {return _selector;}
    inline void setSelector(boss_map::MapRelationSelector* selector_) { 
      _selector= selector_;
      if (_selector)
	_selector->setManager(_manager);
    }
    
    void serialize(boss::ObjectData& data, boss::IdContext& context);
    void deserialize(boss::ObjectData& data, boss::IdContext& context);
    
    std::set<MapNodeRelation*>& relations() {return _relations;}
    std::map<int, MapNode*>& keyNodes() {return _keyNodes;}
    std::list<MapNodeBinaryRelation*>& committedRelations() {return _committedRelations;}
    std::list<MapNodeBinaryRelation*>& candidateRelations() {return _candidateRelations;}
    std::vector<std::set<MapNode*> >& partitions() {return _partitions;}
    std::set<MapNode*>* currentPartition() {return _currentPartition;}
    virtual void process(Serializable* s);
    virtual void flush();
    virtual void setManager(MapManager* manager);
    inline MapManager* manager() const { return _manager;}
    float _consensusInlierTranslationalThreshold;
    float _consensusInlierRotationalThreshold;
    int    _consensusMinTimesCheckedThreshold;
    bool autoProcess;
    bool _debug;
  protected:
    void validatePartitions(std::set<MapNode*>& other, 
			    std::set<MapNode*>& current);
    std::vector<std::set<MapNode*> > _partitions;
    std::set<MapNode*>* _currentPartition;

    MapNode* _pendingTrackerFrame, *_lastTrackerFrame;
    boss_map::MapManager* _manager;
    PoseAcceptanceCriterion* _criterion;
    MapRelationSelector* _selector;
    std::list<MapNodeBinaryRelation*> _results;
    std::list<MapNodeBinaryRelation*> _committedRelations;
    std::list<MapNodeBinaryRelation*> _candidateRelations;
    std::list<Serializable*>  _outputQueue;
    std::set<MapNodeRelation*> _relations;
    std::map<int, MapNode*> _keyNodes;

  };


  class KeyNodeAcceptanceCriterion: public PoseAcceptanceCriterion{
  public:
    KeyNodeAcceptanceCriterion(MapCloser* closer_=0, 
			       MapManager* manager_=0, 
			       PoseAcceptanceCriterion* otherCriterion=0,
			       int id = -1, boss::IdContext* context = 0);
    void setReferencePose(const Eigen::Isometry3d& pose_);
    virtual bool accept(MapNode* n);

    MapCloser* closer() {return _closer;}
    void setCloser(MapCloser* closer_) {_closer = closer_;}

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

  protected:
    MapCloser* _closer;
    PoseAcceptanceCriterion* _otherCriterion;
  };


  class MapCloserActiveRelationSelector: public MapRelationSelector {
  public:
    MapCloserActiveRelationSelector(MapCloser* closer=0, 
				    MapManager* manager=0, 
				    int id = -1, 
				    boss::IdContext* context = 0);
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

    virtual bool accept(MapNodeRelation* r);
    MapCloser* closer() {return _closer;}
    void setCloser(MapCloser* closer_) {_closer = closer_;}
  protected:
    MapCloser* _closer;
  };

}
