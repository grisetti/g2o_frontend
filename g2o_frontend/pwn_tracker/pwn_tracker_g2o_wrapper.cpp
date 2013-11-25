#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

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

#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss_map/boss_map_g2o_reflector.h"
#include <fstream>
#include <iostream>
#include <queue>
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "cache.h"
#include "pwn_tracker_g2o_wrapper.h"

namespace pwn_tracker {

  G2oWrapper::G2oWrapper(MapManager* manager){
    this->manager = manager;
    graph=g2oInit();
    reflector = new MapG2OReflector(manager,graph);
    manager->actionHandlers().push_back(reflector);
    reflector->init();
  }

  void G2oWrapper::optimize(){
    cerr << "optimizing" << endl;
    // scan for all edges that are of type PwnTrackerRelation that are not active and remove them from the optimization
    OptimizableGraph::EdgeSet eset;
    cerr << "total number of relations: " << manager->relations().size() << endl;
    for (std::set<MapNodeRelation*>::iterator it=manager->relations().begin(); it!=manager->relations().end(); it++){
      g2o::EdgeSE3* e=reflector->relation(*it);
      if (!e)
	continue;
      PwnCloserRelation* rel=dynamic_cast<PwnCloserRelation*>(*it);
      if (! rel || (rel && rel->accepted)){
	eset.insert(e);
      } 
    }
    //cerr << "active ones: : " << eset.size() << endl;
    OptimizableGraph::Vertex* gauge = graph->findGauge();
    gauge->setFixed(true);
    graph->initializeOptimization(eset);
    
    cerr << "GLOBAL OPT" << endl;
    graph->setVerbose(false);
    graph->optimize(10);
    //cerr << "copying estimate" << endl;
    reflector->copyEstimatesFromG2O();
    //cerr << "done" << endl;
  }

  SparseOptimizer * G2oWrapper::g2oInit(){
    // graph construction
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmLevenberg* solverGauss   = new OptimizationAlgorithmLevenberg(blockSolver);
    SparseOptimizer * graph = new SparseOptimizer();
    graph->setAlgorithm(solverGauss);
    return graph;
  }

  void G2oWrapper::save(const std::string& filename){
    cerr << "writing g2o file [" << filename << "]" << endl;
    graph->save(filename.c_str());
  }
  
}
