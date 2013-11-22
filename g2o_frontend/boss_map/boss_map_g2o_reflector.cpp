#include "boss_map_g2o_reflector.h"

namespace boss_map {
  using namespace std;
  using namespace boss;

  MapG2OReflector::MapG2OReflector(MapManager* _manager, g2o::OptimizableGraph* graph_):
    MapManagerActionHandler (_manager ){
    _graph = graph_;
    _lastUsedId = 0;
  }

  MapG2OReflector::~MapG2OReflector() {};
  
  void MapG2OReflector::init() {
    _graph->clear();
    for (std::set<MapNode*>::iterator it=_manager->nodes().begin(); it!=_manager->nodes().end(); it++){
      nodeAdded(*it);
    }
    for (std::set<MapNodeRelation*>::iterator it=_manager->relations().begin(); it!=_manager->relations().end(); it++){
      relationAdded(*it);
    }
  }

  
  void MapG2OReflector::copyEstimatesToG2O() {
    for (std::map<MapNode*, g2o::VertexSE3*>::iterator it=_nm2g.begin();
	 it!=_nm2g.end(); it++){
      MapNode* n = it->first;
      g2o::VertexSE3* v = it->second;
      v->setEstimate(n->transform());
    }
  }

  void MapG2OReflector::copyEstimatesFromG2O(){
    for (std::map<MapNode*, g2o::VertexSE3*>::iterator it=_nm2g.begin();
	 it!=_nm2g.end(); it++){
      MapNode* n = it->first;
      g2o::VertexSE3* v = it->second;
      n->setTransform(v->estimate());
    }
  }

  void MapG2OReflector::nodeAdded(MapNode* n) {
    g2o::VertexSE3 * gn = new g2o::VertexSE3;
    gn->setId(_lastUsedId++);
    gn->setEstimate(n->transform());
    _nm2g.insert(make_pair(n,gn));
    _ng2m.insert(make_pair(gn,n));
    _graph->addVertex(gn);
  }
  void MapG2OReflector::nodeRemoved(MapNode* n) {
    g2o::VertexSE3 * gn=_nm2g[n];
    _nm2g.erase(n);
    _ng2m.erase(gn);
      _graph->removeVertex(gn);
  }

  void MapG2OReflector::relationAdded(MapNodeRelation* _r) {
    MapNodeBinaryRelation* r = dynamic_cast<MapNodeBinaryRelation*>(_r);
    if (!r)
      return;
    
    g2o::EdgeSE3* gr = new g2o::EdgeSE3;
    MapNode* n0=r->nodes()[0];
    MapNode* n1=r->nodes()[1];
    gr->setVertex(0,_nm2g[n0]);
    gr->setVertex(1,_nm2g[n1]);
    gr->setMeasurement(r->transform());
    gr->setInformation(r->informationMatrix());
    _rm2g.insert(make_pair(r,gr));
    _rg2m.insert(make_pair(gr,r));
    _graph->addEdge(gr);
  }

  void MapG2OReflector::relationRemoved(MapNodeRelation* _r) {
    MapNodeBinaryRelation* r = dynamic_cast<MapNodeBinaryRelation*>(_r);
    if (!r)
      return;
    g2o::EdgeSE3* gr = _rm2g[r];
    _rm2g.erase(r);
    _rg2m.erase(gr);
    _graph->removeEdge(gr);
  }

  MapNode* MapG2OReflector::node(g2o::VertexSE3* n){
    std::map<g2o::VertexSE3*, MapNode*>::iterator it=_ng2m.find(n);
    if (it==_ng2m.end())
      return 0;
    return it->second;
  }
  
  MapNodeRelation* MapG2OReflector::relation(g2o::EdgeSE3* r){
    std::map<g2o::EdgeSE3*, MapNodeRelation*>::iterator it=_rg2m.find(r);
    if (it==_rg2m.end())
      return 0;
    return it->second;
  }
  
  g2o::VertexSE3* MapG2OReflector::node(MapNode* n){
    std::map<MapNode*, g2o::VertexSE3*>::iterator it=_nm2g.find(n);
    if (it==_nm2g.end())
      return 0;
    return it->second;
  }
  
  g2o::EdgeSE3* MapG2OReflector::relation(MapNodeRelation* r){
    std::map<MapNodeRelation*, g2o::EdgeSE3*>::iterator it=_rm2g.find(r);
    if (it==_rm2g.end())
      return 0;
    return it->second;
  }

}
