#ifndef _BOSS_MAP_G2O_REFLECTOR_H_
#define _BOSS_MAP_G2O_REFLECTOR_H_

#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam3d/types_slam3d.h"

namespace boss_map_building {
  using namespace boss;
  using namespace boss_map;

  class MapG2OReflector: public MapManagerActionHandler {
  public:
    MapG2OReflector(MapManager* _manager, g2o::OptimizableGraph* graph_);
    inline g2o::OptimizableGraph* graph() {return _graph;}
    virtual ~MapG2OReflector();
    void init();
    void copyEstimatesToG2O();
    void copyEstimatesFromG2O();

    virtual void nodeAdded(MapNode* n);
    virtual void nodeRemoved(MapNode* n);
    virtual void relationAdded(MapNodeRelation* _r);
    virtual void relationRemoved(MapNodeRelation* r);
    MapNode* node(g2o::VertexSE3* n);
    MapNodeRelation* relation(g2o::EdgeSE3* r);
    g2o::VertexSE3* node(MapNode* n);
    g2o::EdgeSE3* relation(MapNodeRelation* r);

  protected:
    g2o::OptimizableGraph* _graph;
    std::map<g2o::VertexSE3*, MapNode*> _ng2m;
    std::map<MapNode*, g2o::VertexSE3*> _nm2g;
    std::map<g2o::EdgeSE3*, MapNodeRelation*> _rg2m;
    std::map<MapNodeRelation*, g2o::EdgeSE3*> _rm2g;
    int _lastUsedId;
  };

}

#endif
