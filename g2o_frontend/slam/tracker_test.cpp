#include <signal.h>

#include "feature_tracker.h"
#include "feature_tracker_closure.h"
#include "feature_tracker_pointxy.h"

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

using namespace std;
using namespace g2o;


void dumpEdges(ostream& os, BaseFrameSet& fset, 
	       bool writeOdometry = true, 
	       bool writeLandmarks = true, 
	       bool writeIntraFrame = false) {

   GraphItemSelector graphSelector;
  graphSelector.compute(fset);
  HyperGraph::EdgeSet& eset=graphSelector.selectedEdges();
  OptimizableGraph::VertexSet& vset=graphSelector.selectedVertices();
  
  os << "set size ratio -1" << endl;
  os << "plot ";
  if (writeOdometry) {
    os << "'-' w l lw 0.5, ";
  }
  if (writeIntraFrame) {
    os << "'-' w l lw 0.5, ";
  }
  if (writeLandmarks) {
    os << "'-' w p ps 0.5,";
  }
  os << "'-' w p ps 1.5" << endl;

  if (writeOdometry){
    for (HyperGraph::EdgeSet::iterator it = eset.begin(); it!=eset.end(); it++){
      const EdgeSE2* e = dynamic_cast<const EdgeSE2*>(*it);
      if (! e)
	continue;
      const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(e->vertices()[0]);
      const VertexSE2* v2 = dynamic_cast<const VertexSE2*>(e->vertices()[1]);
      os << v1->estimate().translation().x() << " " << v1->estimate().translation().y() << endl;
      os << v2->estimate().translation().x() << " " << v2->estimate().translation().y() << endl;
      os << endl;
    }
    os << "e" << endl;
  }

  if (writeIntraFrame) {
    for (BaseFrameSet::iterator it = fset.begin(); it!=fset.end(); it++){
      BaseFrame* f = *it;
      const VertexSE2* v1 = f->vertex<const VertexSE2*>();
      if (!v1)
	continue;
      for (BaseFrameSet::iterator iit = f->neighbors().begin(); iit!=f->neighbors().end(); iit++){
	BaseFrame* f2 = *iit;
	if (! fset.count(f2))
	  continue;
	const VertexSE2* v2 = f2->vertex<const VertexSE2*>();
	if (! v2)
	  continue;
	os << v1->estimate().translation().x() << " " << v1->estimate().translation().y() << " " << v1->estimate().rotation().angle() << endl;
	os << v2->estimate().translation().x() << " " << v2->estimate().translation().y() << " " << v2->estimate().rotation().angle() << endl;
	os << endl;
      }
    }
    os << "e" << endl;
  }

  // write the landmarks
  if (writeLandmarks) {
    for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it!=vset.end(); it++){
      const VertexPointXY* v = dynamic_cast<const VertexPointXY*>(*it);
      if (! v)
	continue;
      os << v->estimate().x() << " " << v->estimate().y() <<  endl;
    }
    os << "e" << endl;
  }
  
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it!=vset.end(); it++){
    const VertexSE2* v = dynamic_cast<const VertexSE2*>(*it);
    if (! v)
      continue;
    os << v->estimate().translation().x() << " " << v->estimate().translation().y() << " " << v->estimate().rotation().angle() << endl;
  }
  os << "e" << endl;
  os << endl;
}


void dumpEdges(ostream& os, MapperState* mapperState) {
  GraphItemSelector graphSelector;
  BaseFrameSet fset;
  for (VertexFrameMap::iterator it = mapperState->frames().begin(); it!=mapperState->frames().end(); it++){
    fset.insert(it->second);
  }
  dumpEdges(os, fset);
}




volatile bool hasToStop;
void sigquit_handler(int sig)
{
  if (sig == SIGINT) {
    hasToStop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

void saveThings(string filename , MapperState* mapperState, int seqNum = -1)  {
  if (seqNum>-1) {
    string baseFilename = filename.substr(0,filename.find_last_of("."));
    char seqNumStr[10];
    sprintf(seqNumStr, "%04d", seqNum);
    filename = baseFilename + "-" + seqNumStr + ".g2o";
  }

  OptimizableGraph::VertexSet vset;
  HyperGraph::EdgeSet eset;
  GraphItemSelector graphSelector;
  BaseFrameSet fset;
  for (VertexFrameMap::iterator it = mapperState->frames().begin(); it!=mapperState->frames().end(); it++){
    fset.insert(it->second);
  }
  graphSelector.compute(fset);
  eset=graphSelector.selectedEdges();
  vset=graphSelector.selectedVertices();

  ofstream os(filename.c_str());
  mapperState->graph()->saveSubset(os, eset);
}


int main(int argc, char**argv){
  hasToStop = false;
  string filename;
  string outfilename;
  bool noLoop;
  CommandArgs arg;
  int localMapSize;
  int minFeaturesInCluster;
  int minFrameToFrameInliers;
  int minLandmarkCreationFrames;
  int optimizeEachN;
  int dumpEachN;
  double incrementalGuessMaxFeatureDistance;
  double incrementalRansacInlierThreshold;
  double incrementalDistanceDifferenceThreshold;
  double loopAngularClosureThreshold;
  int    loopGuessMaxFeatureDistance;
  double loopLinearClosureThreshold;
  double loopRansacIdentityMatches;
  double loopRansacInlierThreshold;
  double loopRansacInlierRatio;
  double loopLandmarkMergeDistance;
  int intraFrameConnectivityThreshold;
  int localOptimizeIterations;
  int globalOptimizeIterations;
  int updateViewerEachN;
  float maxRange;
  float minRange;
  bool odometryIsGood;
  int incrementalFeatureTrackingWindow;
  arg.param("o", outfilename, "otest.g2o", "output file name"); 
  arg.param("maxRange", maxRange, 1e3, "maximum range to sense features"); 
  arg.param("minRange", minRange, 0.5, "minimum range to sense features");
  arg.param("minLandmarkCreationFrames", minLandmarkCreationFrames, 2, "minimum range to sense features");
  arg.param("noLoop", noLoop, false, "disable loop closure"); 
  arg.param("odometryIsGood", odometryIsGood, false, "use settings for good odometry"); 
  arg.param("localMapSize", localMapSize, 5, "num of nodes in the path to use in the local map"); 
  arg.param("incrementalFeatureTrackingWindow", incrementalFeatureTrackingWindow, 4, "num of frames to look before to seek for a feature match"); 
  arg.param("incrementalGuessMaxFeatureDistance", incrementalGuessMaxFeatureDistance, 1., "max distance between predicted and projected feature, used to process raw readings"); 
  arg.param("incrementalRansacInlierThreshold", incrementalRansacInlierThreshold, 0.3, "inlier distance in the incremental phase"); 
  arg.param("incrementalDistanceDifferenceThreshold", incrementalDistanceDifferenceThreshold, 0.2, "min distance between an inlier and the second best inlier for the same feature"); 
  
  arg.param("loopLinearClosureThreshold", loopLinearClosureThreshold, 5., "max linear distance between frames to check for loop closure");
  arg.param("loopAngularClosureThreshold", loopAngularClosureThreshold, M_PI, "max angular between frames to check for loop closure");
  arg.param("loopGuessMaxFeatureDistance", loopGuessMaxFeatureDistance, 2., "max distance between predicted and projected feature, used to determine correspondence in loop closing"); 
  arg.param("loopRansacIdentityMatches", loopRansacIdentityMatches, 4, "min number of matches between known landmarks to use for ransac to force the alignment between two local maps"); 
  arg.param("loopRansacInlierThreshold", loopRansacInlierThreshold, .4, "inlier distance threshold for ransac in loop closing"); 
  arg.param("loopRansacInlierRatio", loopRansacInlierRatio, .8, "inlier fraction  for ransac in loop closing"); 
  arg.param("loopLandmarkMergeDistance", loopLandmarkMergeDistance, .1, "distance between two instances of landmarks in the local under which the merging occursx"); 
  
  arg.param("intraFrameConnectivityThreshold", intraFrameConnectivityThreshold, 2, "num of landmarks that should be seen by two frames to consider them neighbors"); 
  arg.param("localOptimizeIterations", localOptimizeIterations, 3, "iterations of the optimizer during the local phase (local maps, incremental matching)"); 
  arg.param("globalOptimizeIterations", globalOptimizeIterations, 10, "iterations of the optimizer when merging landmarks"); 
  arg.param("optimizeEachN", optimizeEachN, 5, "perform a global optimization each <x> frames"); 
  arg.param("dumpEachN", dumpEachN, -1, "saves a dump of the running SLAM problem each <x> iterations"); 
  arg.param("updateViewerEachN", updateViewerEachN, -1, "updates the viewer of the running SLAM problem each <x> iterations"); 

  arg.param("minFeaturesInCluster", minFeaturesInCluster, 10, "min num of features to consider in a cluster "); 
  arg.param("minFrameToFrameInliers", minFrameToFrameInliers, 0, "min num of matching features to do something with the frame"); 
  arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
  
  arg.parseArgs(argc, argv);
  
  // graph construction
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer * graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);
  graph->load(filename.c_str());
  
  // sort the vertices based on the id
  std::vector<int> vertexIds(graph->vertices().size());
  int k=0;
  for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
    vertexIds[k++] = (it->first);
  }

  std::sort(vertexIds.begin(), vertexIds.end());



  PointXYLandmarkConstructor* landmarkConstructor = new PointXYLandmarkConstructor;
  PointXYInitialGuessCorrespondenceFinder* correspondenceFinder = new PointXYInitialGuessCorrespondenceFinder();
  correspondenceFinder->setDistanceThreshold(incrementalGuessMaxFeatureDistance);
  correspondenceFinder->setFeatureMappingMode(0, UseRobotPose);
  correspondenceFinder->setFeatureMappingMode(1, UseRobotPose);

  PointXYRansacMatcher* matcher = new PointXYRansacMatcher;
  matcher->setFeatureMappingMode(0, UseRobotPose);
  matcher->setFeatureMappingMode(1, UseRobotPose);
  matcher->setInlierDistanceThreshold(incrementalRansacInlierThreshold);

  CorrespondenceSecondMatchFilter* secondMatchFilter = new CorrespondenceSecondMatchFilter();
  secondMatchFilter->setDistanceDifferenceThreshold(incrementalDistanceDifferenceThreshold);

  MapperState* mapperState= new MapperState(graph, landmarkConstructor);
  
  Tracker* tracker = new Tracker(mapperState, correspondenceFinder, matcher);
  
  mapperState->setMinLandmarkCreationFrames(minLandmarkCreationFrames);
  //mapperState->setMinLandmarkCommitFrames(2);

  SE2LoopClosureCandidateDetector* closureDetector = new SE2LoopClosureCandidateDetector(mapperState);
  closureDetector->setLinearClosureThreshold(loopLinearClosureThreshold);
  closureDetector->setAngularClosureThreshold(loopAngularClosureThreshold);


  PointXYInitialGuessCorrespondenceFinder* loopCorrespondenceFinder = new PointXYInitialGuessCorrespondenceFinder();
  loopCorrespondenceFinder->setDistanceThreshold(loopGuessMaxFeatureDistance);
  loopCorrespondenceFinder->setFeatureMappingMode(0, UseLandmark);
  loopCorrespondenceFinder->setFeatureMappingMode(1, UseLandmark);

  PointXYRansacMatcher* loopClosureMatcher = new PointXYRansacMatcher;
  loopClosureMatcher->setInlierDistanceThreshold(loopRansacInlierThreshold);
  loopClosureMatcher->setFeatureMappingMode(0, UseLandmark);
  loopClosureMatcher->setFeatureMappingMode(1, UseLandmark);

  LandmarkCorrespondenceManager* landmarkCorrespondenceManager =  new LandmarkCorrespondenceManager(mapperState);
  GraphItemSelector* graphItemSelector = new GraphItemSelector;
  OptimizationManager* optimizationManager = new OptimizationManager(mapperState, graphItemSelector);
  FrameClusterer* frameClusterer = new FrameClusterer;
  PointXYLandmarkDistanceEstimator* landmarkDistanceEstimator = new PointXYLandmarkDistanceEstimator;
  LoopClosureManager* loopClosureManager = new LoopClosureManager(mapperState, 
								  closureDetector,
								  frameClusterer,
								  loopCorrespondenceFinder,
								  landmarkCorrespondenceManager,
								  loopClosureMatcher,
								  optimizationManager,
								  graphItemSelector,
								  landmarkDistanceEstimator);
  loopClosureManager->setMinFeaturesInCluster(minFeaturesInCluster);
  loopClosureManager->setClosureInlierRatio(loopRansacInlierRatio);
  loopClosureManager->setLoopRansacIdentityMatches(loopRansacIdentityMatches);
  loopClosureManager->setLocalOptimizeIterations(localOptimizeIterations);
  loopClosureManager->setLandmarkMergeDistanceThreshold(loopLandmarkMergeDistance);



  cerr << "loaded graph, now trying to insert frames" << endl;
  VertexSE2* vPrev = 0;
  HyperGraph::EdgeSet addedEdges;
  int frameCount = 1;
  int hasToOptimizeGlobally =  false;

  BaseFrame* initialFrame=0;
  signal(SIGINT, sigquit_handler);
  for (size_t i=0; i<vertexIds.size() && ! hasToStop; i++){

    OptimizableGraph::Vertex* _v=graph->vertex(vertexIds[i]);
    VertexSE2* v=dynamic_cast<VertexSE2*>(_v);
    if (!v)
      continue;
    
    EdgeSE2* e = 0;
    if (vPrev) {
      for (OptimizableGraph::EdgeSet::iterator it=v->edges().begin(); it!=v->edges().end(); it++){
	EdgeSE2* _e=dynamic_cast<EdgeSE2*>(*it);
	if (!_e)
	  continue;
	if (_e->vertices()[0]==vPrev || _e->vertices()[1] == vPrev)
	  e = _e;
      }
      if (e) {
	v->setEstimate(vPrev->estimate()*e->measurement());
	addedEdges.insert(e);
      }
    } 

    cerr << "adding frame: " << v->id() << endl;
    mapperState->addFrame(v,e);
    if (! initialFrame)
      initialFrame = mapperState->lastFrame();

    OptimizableGraph::Data* d = v->userData();
    k = 0;
    while(d){
      FeaturePointXYData* fdata = dynamic_cast<FeaturePointXYData*>(d);
      d=d->next();
      if (fdata && fdata->positionMeasurement().norm()<maxRange && fdata->positionMeasurement().norm()>minRange) {
	BaseTrackedFeature* f = new BaseTrackedFeature(mapperState->lastFrame(), fdata, 0);
	mapperState->addTrackedFeature(f);
	k++;
      }
      //cerr << "\t f:" << k++ << endl;
    }
    
    cerr << "adding " << k << " features" << endl;

    int mergedLandmarks = 0;
    BaseFrameSet localFrames;
 
    if (vPrev) {
      // compute the local map, and its origin in localMapGaugeFrame. the gauge is the 
      // oldest frame in the trajectory
      BaseFrame* localMapGaugeFrame = mapperState->lastNFrames(localFrames, localMapSize);

      // do one round of optimization, it never hurts
      //cerr << "A" << endl;
      optimizationManager->initializeLocal(localFrames, localMapGaugeFrame, false);
      optimizationManager->optimize(localOptimizeIterations);
      optimizationManager->cleanup();

      //cerr << "B" << endl;
      
      // look for the conrrespondences based on the odometry
      // thisa applies first the correspondenceFinder than the matcher
      CorrespondenceVector correspondences;
      tracker->searchInitialCorrespondences(correspondences, incrementalFeatureTrackingWindow);
 
      //cerr << "C" << endl;
      
			// look for the conrrespondences based on the odometry
      // thisa applies first the correspondenceFinder than the matcher
      secondMatchFilter->compute(correspondences);

      //cerr << "D" << endl;
      // remove the ambiguous correspondences 
      const CorrespondenceVector& filtered = secondMatchFilter->filtered();
      cerr << "SecondMatchFilter " << filtered.size() << " filtered" << endl;
 
      // if you found a reasonable number of inliers, correct the estimate of the
      // last vertex with the matcher estimate for the translation
      //cerr << "E" << endl;
      SE2 transformation = matcher->transformation();
      if (matcher->numInliers()>=minFrameToFrameInliers){
	v->setEstimate(transformation * v->estimate());
      }

      // update the bookkeeping, by connecting the tracked features found in the correspondences
      // and by creating the landmarks when a track was confirmed for a while
      mapperState->updateTracksAndLandmarks(filtered);
      // do one round of optimization, it makes the local map prettier when looking for loop closures
      //cerr << "F" << endl;
      optimizationManager->initializeLocal(localFrames, localMapGaugeFrame, false);
      optimizationManager->optimize(localOptimizeIterations);
      optimizationManager->cleanup();
      
			//cerr << "G" << endl;
      
      // connect the frames from which you observed some landmark in common
      int newIncrementalLinks = mapperState->refineConnectivity(localFrames, 1, odometryIsGood);
      cerr << "newIncrementalLinks: " << newIncrementalLinks << endl;

      // BaseTrackedFeatureSet newFeaturesWithLandmark;
      // MapperState::selectFeaturesWithLandmarks(newFeaturesWithLandmark, mapperState->lastFrame());
      // cerr << "# features with landmark in the current frame: " << newFeaturesWithLandmark.size() << endl;

      // do the loop closing by matching local maps made out of landmarks and trying to match those
      if (! noLoop && localMapGaugeFrame) {
	loopClosureManager->compute(localFrames, localMapGaugeFrame);
	mergedLandmarks = loopClosureManager->mergedLandmarks();
      }
      hasToOptimizeGlobally = hasToOptimizeGlobally || mergedLandmarks;
    }
    
    cerr << "number of merged landmarks: " << mergedLandmarks << endl;
    cerr << "number of frames interested in this operation: " << loopClosureManager->touchedFrames().size() << endl;

    // again update the bookkeeping on the frame graph
    int newLinks = mapperState->refineConnectivity(loopClosureManager->touchedFrames(), intraFrameConnectivityThreshold);
    cerr << "number of intra-frame links added: " << newLinks << endl;

    // if something happened and enough cycles are passed, do a global optimization)
    if (hasToOptimizeGlobally && !(frameCount%optimizeEachN)){
      cerr << "Global Optimize... ";
      optimizationManager->initializeGlobal(initialFrame);
      optimizationManager->optimize(globalOptimizeIterations);
      optimizationManager->cleanup();
      cerr << "done" << endl;
      hasToOptimizeGlobally =  false;
    }

    if (dumpEachN>0 && !(frameCount%dumpEachN)){
      cerr << "saving... ";
      saveThings(outfilename, mapperState, frameCount);
    }
    if (updateViewerEachN>0 && !(frameCount%updateViewerEachN)){
      dumpEdges(cout, mapperState);
    }

    frameCount ++;

    vPrev = v;
    // int a;
    // cin >> a;

    // do the cleaning
    if ((int)localFrames.size() == localMapSize ) {
      BaseFrame* oldestFrame = 0;
      BaseFrame* outOfLocalMapFrame = 0; 
      for (BaseFrameSet::iterator it = localFrames.begin(); it!= localFrames.end(); it++){
	BaseFrame* f = *it;
	if (! oldestFrame || oldestFrame->vertex<OptimizableGraph::Vertex*>()->id() > oldestFrame->vertex<OptimizableGraph::Vertex*>()->id()){
	  oldestFrame = f;
	}
      }
      if (oldestFrame)
	outOfLocalMapFrame = oldestFrame->previous();
      int killedFeatures = 0;
      if (outOfLocalMapFrame) {
	BaseTrackedFeatureSet fset = outOfLocalMapFrame->features();
	for (BaseTrackedFeatureSet::iterator it=fset.begin(); it!=fset.end(); it++) {
	  BaseTrackedFeature * f = *it;
	  if (! f->landmark()){
	    killedFeatures += mapperState->removeTrackedFeature(f, true);
	  } 
	}
      }
      cerr << "removing " << killedFeatures << " unmatched features from the history" << endl;
    }
  }
  cerr << endl;

  
 
  cerr << "saving.... " << endl;
  saveThings(outfilename, mapperState);
  cerr << "# frames: " << mapperState->frames().size() << endl;
  cerr << "# landmarks: " << mapperState->landmarks().size() << endl;
  cerr << "# vertices: " << graph->vertices().size() << endl;
  cerr << "# edges: " << graph->edges().size() << endl;
  cerr << "# closureCorrespondences: " << landmarkCorrespondenceManager->size() << endl;
  
  cerr << "cleaning up" << endl;
  cerr << "landmarkConstructor" << endl;
  delete landmarkConstructor;
  cerr << "correspondenceFinder" << endl;
  delete correspondenceFinder;
  cerr << "matcher" << endl;
  delete matcher;
  cerr << "secondMatchFilter" << endl;
  delete secondMatchFilter;
  cerr << "tracker" << endl;
  delete tracker;
  cerr << "closureDetector" << endl;
  delete closureDetector;
  cerr << "loopCorrespondenceFinder" << endl;
  delete loopCorrespondenceFinder;
  cerr << "loopClosureMatcher" << endl;
  delete loopClosureMatcher;
  cerr << "landmarkCorrespondenceManager" << endl;
  delete landmarkCorrespondenceManager;
  cerr << "graphItemSelector" << endl;
  delete graphItemSelector;
  cerr << "optimizationManager" << endl;
  delete optimizationManager;
  cerr << "frameClusterer" << endl;
  delete frameClusterer;
  cerr << "landmarkDistanceEstimator" << endl;
  delete landmarkDistanceEstimator;
  cerr << "loopClosureManager" << endl;
  delete loopClosureManager;
  cerr << "mapperState" << endl;
  delete mapperState;
  cerr << "graph" << endl;
  delete graph;
  cerr << "done" << endl;
}
