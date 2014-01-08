#pragma once

#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/stream_processor.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/sparse_optimizer.h"

namespace boss_map_building {
  using namespace boss;
  using namespace boss_map;

  
  class MapG2OReflector: public MapManagerActionHandler {
  public:
    MapG2OReflector(MapManager* _manager=0, g2o::SparseOptimizer* graph_=0, int id=-1, boss::IdContext* context=0);
    inline g2o::OptimizableGraph* graph() {return _graph;}
    virtual ~MapG2OReflector();
    void init();
    void copyEstimatesToG2O();
    void copyEstimatesFromG2O();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
   
    g2o::SparseOptimizer * g2oInit();
    void optimize();
    
    virtual void nodeAdded(MapNode* n);
    virtual void nodeRemoved(MapNode* n);
    virtual void relationAdded(MapNodeRelation* _r);
    virtual void relationRemoved(MapNodeRelation* r);
    MapNode* node(g2o::VertexSE3* n);
    MapNodeRelation* relation(g2o::OptimizableGraph::Edge* r);
    g2o::VertexSE3* node(MapNode* n);
    g2o::OptimizableGraph::Edge* relation(MapNodeRelation* r);

    MapRelationSelector* selector() {return _selector;}
    void setSelector(MapRelationSelector* selector_) {_selector = selector_;}
  protected:
    g2o::SparseOptimizer* _graph;
    std::map<g2o::VertexSE3*, MapNode*> _ng2m;
    std::map<MapNode*, g2o::VertexSE3*> _nm2g;
    std::map<g2o::OptimizableGraph::Edge*, MapNodeRelation*> _rg2m;
    std::map<MapNodeRelation*, g2o::OptimizableGraph::Edge*> _rm2g;
    int _lastUsedId;
    MapRelationSelector* _selector;
  };

  class OptimizerProcessor: public StreamProcessor {
  public:
    OptimizerProcessor(int id=-1, boss::IdContext* context = 0);

    inline MapManager* manager() { return  _manager; }

    inline void setManager(MapManager* manager_) { _manager = manager_; }

    inline MapG2OReflector* optimizer() {return _optimizer;}

    inline void setOptimizer(MapG2OReflector* optimizer_) { _optimizer = optimizer_; }

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);

    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

    virtual void process(Serializable* s);

  protected:
    MapManager* _manager;
    MapG2OReflector* _optimizer;
  };

}
