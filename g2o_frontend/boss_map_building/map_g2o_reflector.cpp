#include "map_g2o_reflector.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

namespace boss_map_building {
  using namespace std;
  using namespace boss;
  using namespace g2o;

  MapG2OReflector::MapG2OReflector(MapManager* _manager, g2o::SparseOptimizer* graph_):
    MapManagerActionHandler (_manager ){
    if (! graph_) {
      _graph = g2oInit();
    } else 
      _graph = graph_;
    _lastUsedId = 0;
    init();
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
    {
      MapNodeBinaryRelation* r = dynamic_cast<MapNodeBinaryRelation*>(_r);
      if  (r) {    
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
	return;
      }
    }
    {
      MapNodeUnaryRelation* r = dynamic_cast<MapNodeUnaryRelation*>(_r);
      if  (r) {    
	EdgeSE3Prior* gr = new g2o::EdgeSE3Prior;
	MapNode* n0=r->nodes()[0];
	gr->setVertex(0,_nm2g[n0]);
	gr->setMeasurement(r->transform());
	gr->setInformation(r->informationMatrix());
	_rm2g.insert(make_pair(r,gr));
	_rg2m.insert(make_pair(gr,r));
	_graph->addEdge(gr);
	return;
      }
    }
  }

  void MapG2OReflector::relationRemoved(MapNodeRelation* _r) {
    MapNodeBinaryRelation* r = dynamic_cast<MapNodeBinaryRelation*>(_r);
    if (!r)
      return;
    g2o::OptimizableGraph::Edge* gr = _rm2g[r];
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
  
  MapNodeRelation* MapG2OReflector::relation(g2o::OptimizableGraph::Edge* r){
    std::map<g2o::OptimizableGraph::Edge*, MapNodeRelation*>::iterator it=_rg2m.find(r);
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
  
  g2o::OptimizableGraph::Edge* MapG2OReflector::relation(MapNodeRelation* r){
    std::map<MapNodeRelation*, g2o::OptimizableGraph::Edge*>::iterator it=_rm2g.find(r);
    if (it==_rm2g.end())
      return 0;
    return it->second;
  }


  void MapG2OReflector::optimize(){
    copyEstimatesToG2O();
    cerr << "optimizing" << endl;
    // scan for all edges that are of type PwnTrackerRelation that are not active and remove them from the optimization
    g2o::OptimizableGraph::EdgeSet eset;
    cerr << "total number of relations: " << _manager->relations().size() << endl;
    for (std::set<MapNodeRelation*>::iterator it=_manager->relations().begin(); it!=_manager->relations().end(); it++){
      g2o::OptimizableGraph::Edge* e=relation(*it);
      if (!e)
	continue;
      if (! _selector || _selector->accept(*it)){
	eset.insert(e);
      } 
    }
    
    cerr << "active ones: : " << eset.size() << endl;
    g2o::OptimizableGraph::Vertex* gauge = _graph->findGauge();
    gauge->setFixed(true);
    _graph->initializeOptimization(eset);
    
    cerr << "GLOBAL OPT: " << gauge->id() << endl;
    g2o::VertexSE3* vg = (g2o::VertexSE3*)gauge;
    cerr << "T0: " << t2v(vg->estimate()).transpose() << endl;

    //_graph->setVerbose(true);
    _graph->optimize(10);
    /*
    for (size_t i = 0; i<_graph->activeVertices().size(); i++){
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(_graph->activeVertices()[i]);
      if (!v)
	continue;
      MapNode* n = node(v);
      n->setTransform(v->estimate());
      if (v == gauge)
	cerr << "T0_copied: " << t2v(n->transform()).transpose() << endl;
    }
    */
    gauge->setFixed(false);
    //cerr << "copying estimate" << endl;
    copyEstimatesFromG2O();
    //cerr << "done" << endl;
  }

  g2o::SparseOptimizer * MapG2OReflector::g2oInit(){
    // graph construction
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    //OptimizationAlgorithmLevenberg* solverGauss   = new OptimizationAlgorithmLevenberg(blockSolver);
    OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
    SparseOptimizer * graph = new SparseOptimizer();
    graph->setAlgorithm(solverGauss);
    return graph;
  }

}
