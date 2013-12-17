#pragma once

#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/sparse_optimizer.h"

namespace boss_map_building {
  using namespace boss;
  using namespace boss_map;

  class MapG2OReflector: public MapManagerActionHandler {
  public:
    MapG2OReflector(MapManager* _manager, g2o::SparseOptimizer* graph_=0);
    inline g2o::OptimizableGraph* graph() {return _graph;}
    virtual ~MapG2OReflector();
    void init();
    void copyEstimatesToG2O();
    void copyEstimatesFromG2O();
    
    g2o::SparseOptimizer * g2oInit();
    void optimize();
    
    virtual void nodeAdded(MapNode* n);
    virtual void nodeRemoved(MapNode* n);
    virtual void relationAdded(MapNodeRelation* _r);
    virtual void relationRemoved(MapNodeRelation* r);
    MapNode* node(g2o::VertexSE3* n);
    MapNodeRelation* relation(g2o::EdgeSE3* r);
    g2o::VertexSE3* node(MapNode* n);
    g2o::EdgeSE3* relation(MapNodeRelation* r);

    MapRelationSelector* selector() {return _selector;}
    void setSelector(MapRelationSelector* selector_) {_selector = selector_;}
  protected:
    g2o::SparseOptimizer* _graph;
    std::map<g2o::VertexSE3*, MapNode*> _ng2m;
    std::map<MapNode*, g2o::VertexSE3*> _nm2g;
    std::map<g2o::EdgeSE3*, MapNodeRelation*> _rg2m;
    std::map<MapNodeRelation*, g2o::EdgeSE3*> _rm2g;
    int _lastUsedId;
    MapRelationSelector* _selector;
  };

}
