
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
#include "pwn_closer.h"
#include "pwn_tracker_g2o_wrapper.h"

using namespace std;
using namespace g2o;
using namespace pwn;
using namespace boss;
using namespace boss_map;
using namespace pwn_tracker;


/*
struct G2oWrapper{
  G2oWrapper(MapManager* manager){
    this->manager = manager;
    graph=g2oInit();
    reflector = new MapG2OReflector(manager,graph);
    manager->actionHandlers().push_back(reflector);
    reflector->init();
  }

  void optimize(){
    cerr << "optimizing" << endl;
    // scan for all edges that are of type PwnTrackerRelation that are not active and remove them from the optimization
    OptimizableGraph::EdgeSet eset;
    cerr << "total number of relations: " << manager->relations().size() << endl;
    for (std::set<MapNodeRelation*>::iterator it=manager->relations().begin(); it!=manager->relations().end(); it++){
      g2o::EdgeSE3* e=reflector->relation(*it);
      if (!e)
	continue;
      PwnCloserRelation* rel=dynamic_cast<PwnCloserRelation*>(*it);
      if (! rel || rel && rel->accepted){
	  eset.insert(e);
      } 
    }
    cerr << "active ones: : " << eset.size() << endl;
    OptimizableGraph::Vertex* gauge = graph->findGauge();
    gauge->setFixed(true);
    graph->initializeOptimization(eset);
    
    graph->setVerbose(true);
    graph->optimize(10);
    cerr << "copying estimate" << endl;
    reflector->copyEstimatesFromG2O();
    cerr << "done" << endl;
  }

  SparseOptimizer * g2oInit(){
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

  void save(const std::string& filename){
    cerr << "writing g2o file [" << filename << "]" << endl;
    graph->save(filename.c_str());
  }
  
  MapManager* manager;
  MapG2OReflector * reflector;
  SparseOptimizer * graph;
};
*/

MapManager* load(std::vector<Serializable*>& objects,
		 Deserializer& des){
  Serializable* o=0;
  while( (o=des.readObject()) ){
    objects.push_back(o);
    boss_map::MapManager* m = dynamic_cast<boss_map::MapManager*>(o);
    if (m) {
      return m;
    }
  }
  return 0;
}




int main(int argc, char** argv){
  if (argc<3) {
    cerr << " u should provide a config file and an input file" << endl;
    return 0;
  }
  pwn::Aligner* aligner;  pwn::DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;
  
  Deserializer des;
  des.setFilePath(argv[2]);
  // load the log
  std::vector<boss::Serializable*> objects;

  MapManager* manager = load(objects, des);
  if (! manager) 
    return 0;

  // install the optimization wrapper
  G2oWrapper* wrapper = new G2oWrapper(manager);

  // create a closer
  PwnCloser* closer = new PwnCloser(aligner,converter,manager);
  closer->setScale(4);
  
  // install a closure criterion to select nodes in the clser
  DistancePoseAcceptanceCriterion criterion(manager);
  criterion.setRotationalDistance(M_PI/4);
  criterion.setTranslationalDistance(1);
  closer->setCriterion(&criterion);


  // sequentially process the data
  boss::Serializable* o=0;
  size_t lastAddedResult = 0;
  int count=0;
  while( (o=des.readObject()) ){
    objects.push_back(o);
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(o);
    if (f) {
      closer->addFrame(f);
      count++;
    }
    PwnTrackerRelation* r = dynamic_cast<PwnTrackerRelation*>(o);
    if (r) {
      closer->addRelation(r);
    }
    int committedRelations = closer->committedRelations();
    // for (size_t k = lastAddedResult; k<closer->results().size(); k++){
    //   MatchingResult& result = closer->results()[k];
    //   //if (result.relation->accepted)
    // 	newRelations ++;
    //   // if (result.nonZeros< 3000 || result.outliers>100 || result.inliers<1000)
    //   //  	continue;
    //   // char fname[1024];
    //   // sprintf(fname, "match-%05d-%05d-%06d-%06d.pgm",result.from->seq, result.to->seq, result.inliers, result.outliers);
    //   // cerr << "k: " << k <<  " [" << fname << "]" <<  endl;
    //   // cv::Mat m=result.diffRegistered;
    //   // cv::imwrite(fname, m);
    //   // manager->addRelation(result.relation);
    // }
    lastAddedResult = closer->results().size();
    if (committedRelations ){
      char fname[100];
      wrapper->optimize();
      sprintf(fname, "out-%05d.g2o", count);
      wrapper->save(fname);
    }
  }
  
  // cerr << "detected closures:" << closer->results().size() << endl;
  // for (size_t i =0; i<closer->results().size(); i++){
  //   MatchingResult& result = closer->results()[i];
  //   if (result.nonZeros< 5000 || result.outliers>100 || result.inliers<2000)
  //     continue;
  //   char fname[1024];
  //   sprintf(fname, "match-%05d-%05d-%06d-%06d.pgm",result.from->seq, result.to->seq, result.inliers, result.outliers);
  //   cerr << "i: " << i <<  " [" << fname << "]" <<  endl;
  //   cv::Mat m=result.diffRegistered;
  //   cv::imwrite(fname, m);
  //   manager->addRelation(result.relation);
  // }

  // cerr << "initializing reflector" << endl;
  // wrapper->save("output.g2o");
  // wrapper->optimize();

  cerr << "writing out" << endl;
  Serializer ser;
  ser.setFilePath("output.log");
  cerr << "writing out fixed input" << endl;
  for (size_t i = 0; i<objects.size(); i++)
    ser.writeObject(*objects[i]);
  return 0;
}
