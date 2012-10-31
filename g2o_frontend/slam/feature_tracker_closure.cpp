#include "feature_tracker_closure.h"
#include "g2o/stuff/timeutil.h"
#include <deque>
#include <assert.h>
#include <deque>

#include "g2o/core/sparse_optimizer.h"

namespace g2o {
  using namespace std;

  LoopClosureCandidateDetector::LoopClosureCandidateDetector(MapperState* mapperState_) {
    _mapperState = mapperState_;
  }


  GraphItemSelector::GraphItemSelector() {
  }

  void GraphItemSelector::compute(BaseFrameSet& frameSet) {
    _selectedEdges.clear();
    _selectedVertices.clear();
    for (BaseFrameSet::iterator it=frameSet.begin(); it!=frameSet.end(); it++){
      BaseSequentialFrame* frame = *it;
      OptimizableGraph::Vertex* v=frame->vertex<OptimizableGraph::Vertex*>();
      _selectedVertices.insert(v);
      for (MatchableIdMap::iterator fit=frame->featuresBegin(); fit != frame->featuresEnd(); fit ++){
	BaseTrackedFeature* feature = reinterpret_cast<BaseTrackedFeature*>(fit->second);
	if (feature->landmark()){
	  OptimizableGraph::Edge* edge = feature->edge<OptimizableGraph::Edge*>();
	  if (edge) {
	    _selectedEdges.insert(edge);
	    for (size_t i=0; i<edge->vertices().size(); i++){
	      _selectedVertices.insert(edge->vertices()[i]);
	    }
	  }
	}
      }
      BaseSequentialFrame* previousFrame = frame->previous();
      if (frameSet.count(previousFrame)){
	OptimizableGraph::Edge* odometryEdge = frame->odometryEdge<OptimizableGraph::Edge*>();
	_selectedEdges.insert(odometryEdge);
      }
    }
  }

  typedef std::deque<BaseSequentialFrame*> BaseFrameDeque;

  void FrameClusterer::compute(BaseFrameSet& frameSet){
    _clusters.clear();
    BaseFrameSet openFrames =  frameSet;
    while(!openFrames.empty()) {
      BaseFrameSet currentCluster;
      BaseSequentialFrame* firstFrame = *(openFrames.begin());
      BaseFrameDeque frameDeque;

      frameDeque.push_back(firstFrame);
      openFrames.erase(firstFrame);
      currentCluster.insert(firstFrame);
      while (! frameDeque.empty()){
	BaseSequentialFrame* frame = frameDeque.front();
	frameDeque.pop_front();
	currentCluster.insert(frame);
	for (BaseFrameSet::iterator neighborIt=frame->neighbors().begin();
	     neighborIt!=frame->neighbors().end(); neighborIt++){
	  BaseSequentialFrame* otherFrame = *neighborIt;
	  if (openFrames.count(otherFrame)){
	    openFrames.erase(otherFrame);
	    currentCluster.insert(otherFrame);
	    frameDeque.push_back(otherFrame);
	  }
	}
      }
      _clusters.push_back(currentCluster);
    }
  }

  OptimizationManager::OptimizationManager(MapperState* mapperState_, GraphItemSelector* graphItemSelector_) {
    _mapperState = mapperState_;
    _graphItemSelector = graphItemSelector_;
    _pushDone = false;
    _gauge = 0;
    _isInitialized = false;
  }
  
  void OptimizationManager::initializeLocal(BaseFrameSet& fset, BaseSequentialFrame* gaugeFrame, bool push){
    _initialize(fset, push, gaugeFrame);
  }

  void OptimizationManager::initializeGlobal(BaseSequentialFrame* gaugeFrame){
    BaseFrameSet fset;
    for (VertexFrameMap::iterator it = _mapperState->frames().begin(); it!=_mapperState->frames().end(); it++){
      fset.insert(it->second);
    }
    _initialize(fset, false, gaugeFrame);
  }
  
  void OptimizationManager::optimize(int iterations){
    if (_edges.empty())
      return;
    SparseOptimizer* optimizer = dynamic_cast <SparseOptimizer*> (_mapperState->graph());
    if (_gauge)
      _gauge->setFixed(true);
    optimizer->initializeOptimization(_edges);
    optimizer->optimize(iterations);
    if (_gauge)
      _gauge->setFixed(false);
  }
  
  void OptimizationManager::cleanup(){
    if (!_isInitialized){
      cerr << "OptimizationManager: Fatal, double cleanup" << endl;
      exit(0);
    }
    _isInitialized = false;
    if (_pushDone) {
      for (OptimizableGraph::VertexSet::iterator it=_vertices.begin(); it!=_vertices.end(); it++){
	OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(*it);
	v->pop();
      }
      _pushDone =  false;
    }
    _edges.clear();
    _vertices.clear();
    _gauge=0;
  }

  void OptimizationManager::_initialize(BaseFrameSet& fset, bool push, BaseSequentialFrame* gaugeFrame){
    if (_isInitialized){
      cerr << "OptimizationManager: Fatal, double initialization" << endl;
      exit(0);
    }
    _isInitialized = true;

    if (_pushDone)  {
      cerr << "FATAL, double push detected" << endl;
    }
    _pushDone = push;
    _edges.clear();
    _vertices.clear();
    _gauge = 0;
    if (fset.empty())
      return;
    _graphItemSelector->compute(fset);
    if (! gaugeFrame) {
      gaugeFrame = *(fset.begin());
    } 
    _gauge = gaugeFrame->vertex<OptimizableGraph::Vertex*>();
    _edges = _graphItemSelector->selectedEdges();
    _vertices = _graphItemSelector->selectedVertices();
    if (push) {
      for (OptimizableGraph::VertexSet::iterator it=_vertices.begin(); it!=_vertices.end(); it++){
	OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(*it);
	v->push();
      }
    }
  }

  bool LandmarkDistanceEstimator::compute(double& , BaseTrackedLandmark* , BaseTrackedLandmark* ) {
    return false;
  }


  LoopClosureManager::LoopClosureManager(MapperState* mapperState_, 
					 LoopClosureCandidateDetector * closureCandidateDetector_,
					 FrameClusterer* frameClusterer_,
					 CorrespondenceFinder* correspondenceFinder_,
					 LandmarkCorrespondenceManager* correspondenceManager_,
					 Matcher* matcher_,
					 OptimizationManager* optimizationManager_,
					 GraphItemSelector* graphItemSelector_,
					 LandmarkDistanceEstimator* landmarkDistanceEstimator_) {
    _mapperState = mapperState_; 
    _closureCandidateDetector = closureCandidateDetector_;
    _frameClusterer = frameClusterer_;
    _correspondenceFinder = correspondenceFinder_;
    _correspondenceManager = correspondenceManager_;
    _matcher = matcher_;
    _optimizationManager = optimizationManager_;
    _graphItemSelector = graphItemSelector_;
    _landmarkDistanceEstimator = landmarkDistanceEstimator_;

    _minFeaturesInCluster = 5 ;
    _closureInlierRatio = 0.5;
    _loopRansacIdentityMatches = 5;
    _localOptimizeIterations = 4;
    _landmarkMergeDistanceThreshold = 0.1;
  }

  void LoopClosureManager::compute(BaseFrameSet& localFrames, BaseSequentialFrame* localMapGaugeFrame) {
    _touchedFrames.clear();
    _mergedLandmarks = 0;
    _touchedFrames.insert(localFrames.begin(), localFrames.end());

    BaseFrameSet closureCandidates;
    if (localMapGaugeFrame) {
      _closureCandidateDetector->compute(_mapperState->lastFrame());
      closureCandidates = _closureCandidateDetector->candidates();
      _touchedFrames.insert(closureCandidates.begin(), closureCandidates.end());

      cerr <<  "local map has" << localFrames.size() << endl;
      // BaseTrackedFeatureSet featuresToMatchInLocalMap;
      // MapperState::selectFeaturesWithLandmarks(featuresToMatchInLocalMap, localFrames);
      // cerr << "found " << featuresToMatchInLocalMap.size() << " features in the local map" << endl;

      MatchableIdMap landmarksToMatchInLocalMap;
      MapperState::selectLandmarks(landmarksToMatchInLocalMap, localFrames);
      cerr << "found " << landmarksToMatchInLocalMap.size() << " landmarks in the local map" << endl;

      BaseFrameSet prunedClosures;
      std::set_difference(closureCandidates.begin(), closureCandidates.end(), 
			  localFrames.begin(), localFrames.end(),
			  inserter(prunedClosures, prunedClosures.begin()));
      cerr << "pruned closures: " << prunedClosures.size() << endl;
      //FrameClusterer frameClusterer;
      _frameClusterer->compute(prunedClosures);
      cerr << "in the closure there are " << _frameClusterer->numClusters() << " regions" << endl;
      for (int i =0; i < _frameClusterer->numClusters() && landmarksToMatchInLocalMap.size(); i++ ){
	cerr << "\t cluster: " << i << " " << _frameClusterer->cluster(i).size() << endl;
  
	MatchableIdMap landmarksInCluster;
	MapperState::selectLandmarks(landmarksInCluster, _frameClusterer->cluster(i));

	int minFeatures = (int)landmarksInCluster.size() < (int) landmarksToMatchInLocalMap.size() ? 
	  (int)landmarksInCluster.size()  : (int) landmarksToMatchInLocalMap.size();

	if (minFeatures < _minFeaturesInCluster){
	  //cerr << "\t\t" << " #too few features (" << minFeatures << "), rejecting match"<<  endl;
	} else {
	  cerr << "\t\t" << " #enough features  (" << minFeatures << "), accepting match"<<  endl;
	  // optimize each cluster
	  _optimizationManager->initializeLocal(_frameClusterer->cluster(i),0,true);
	  _optimizationManager->optimize(_localOptimizeIterations);
	  
	  _correspondenceFinder->compute(landmarksInCluster.begin(), landmarksInCluster.end(),
					 landmarksToMatchInLocalMap.begin(), landmarksToMatchInLocalMap.end());
	  CorrespondenceVector clusterClosureCorrespondences=_correspondenceFinder->correspondences();
	  cerr << "\t\t" << " #matches:  " << clusterClosureCorrespondences.size() <<  endl;
	  
	  _matcher->compute(clusterClosureCorrespondences);
	  CorrespondenceVector clusterClosureMatches=_matcher->matches();
	  int numDifferentMatches = clusterClosureMatches.size();

	  float inlierRatio = (float) numDifferentMatches / minFeatures;
	  int landmarkIdentityMatches = _matcher->landmarkIdentityMatches();
	  cerr << "\t\t" << " #ransac:   " << clusterClosureMatches.size() 
	       <<  " differentMatches: " <<  numDifferentMatches << "  inlierRatio: " << inlierRatio << " identityMatches: " << 
	    landmarkIdentityMatches << endl;
	  
	  if (inlierRatio > _closureInlierRatio || landmarkIdentityMatches > _loopRansacIdentityMatches) {
	    for (size_t k=0; k<clusterClosureMatches.size(); k++){
	      Correspondence c = clusterClosureMatches[k];
	      BaseTrackedLandmark* l1=dynamic_cast<BaseTrackedLandmark*>(c.f1);
	      BaseTrackedLandmark* l2=dynamic_cast<BaseTrackedLandmark*>(c.f2);
	      if (l1!=l2) {
		_correspondenceManager->addCorrespondence(l1, l2);
	      }
	      //cerr << "\t\t\tc:" << k << " " << l1 << " " <<  l2 << " " << c.distance << " " << occurrences << endl;
	    }
	  }
	  _optimizationManager->cleanup();
	} 
      }
    }

    _mergedLandmarks = _correspondenceManager->merge(1);
    int recursivelyMergedLandmarks = 0;
    if (_mergedLandmarks) {
      int newlyMerged;
      do {
	_frameClusterer->compute(closureCandidates);
	newlyMerged = 0;
	for (int i=0; i< _frameClusterer->numClusters(); i++) {
	  _optimizationManager->initializeLocal(_frameClusterer->cluster(i),0,true);
	  _optimizationManager->optimize(_localOptimizeIterations);
	  MatchableIdMap landmarksInCluster;
	  MapperState::selectLandmarks(landmarksInCluster, _frameClusterer->cluster(i));
	  for (MatchableIdMap::iterator it = landmarksInCluster.begin(); it!=landmarksInCluster.end(); it++){	    BaseTrackedLandmark* l1 = reinterpret_cast<BaseTrackedLandmark*>(it->second);
	    for (MatchableIdMap::iterator iit = it; iit!=landmarksInCluster.end(); iit++) {
	      BaseTrackedLandmark* l2 = reinterpret_cast<BaseTrackedLandmark*>(iit->second);
	      if (l1 == l2)
		continue;
	      double distance;
	      if(_landmarkDistanceEstimator->compute(distance, l1, l2) && distance < _landmarkMergeDistanceThreshold){
		_correspondenceManager->addCorrespondence(l1,l2);
	      }
	    }
	  }
	  _optimizationManager->cleanup();
	  newlyMerged += _correspondenceManager->merge(1);
	}
	recursivelyMergedLandmarks += newlyMerged;
      } while (newlyMerged);
    }
    _mergedLandmarks += recursivelyMergedLandmarks;
  }

  LandmarkCorrespondence::LandmarkCorrespondence(BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_) {
    if (l1_<=l2_){
      l1 = l1_;
      l2 = l2_;
    } else { 
      l2 = l1_;
      l1 = l2_;
    }
  }
  

  LandmarkCorrespondenceManager::LandmarkCorrespondenceManager(MapperState* mapperState_) {
    _mapperState = mapperState_;
  }

  int LandmarkCorrespondenceManager::addCorrespondence(BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_, int k){
    LandmarkCorrespondence corr(l1_, l2_);
    LandmarkCorrespondenceIntMap::iterator it=_landmarkCorrespondenceMap.find(corr);
    if (it==_landmarkCorrespondenceMap.end()){
      _landmarkCorrespondenceMap.insert(make_pair(corr, k));
      return 1;
    } else {
      it->second +=k;
      return it->second;
    }
  }
  
  bool LandmarkCorrespondenceManager::removeCorrespondence(BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_) {
    LandmarkCorrespondence corr(l1_, l2_);
    LandmarkCorrespondenceIntMap::iterator it=_landmarkCorrespondenceMap.find(corr);
    if (it==_landmarkCorrespondenceMap.end())
      return false;
    _landmarkCorrespondenceMap.erase(it);
    return true;
  }
  
  int LandmarkCorrespondenceManager::occurrences (BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_) {
    LandmarkCorrespondence corr(l1_, l2_);
    LandmarkCorrespondenceIntMap::iterator it=_landmarkCorrespondenceMap.find(corr);
    if (it==_landmarkCorrespondenceMap.end())
      return 0;
    return it->second;
  }

  void LandmarkCorrespondenceManager::mergeLandmarks(BaseTrackedLandmark* lkept, BaseTrackedLandmark* lremoved) {
    int occ=occurrences(lkept, lremoved);
    if (! occ)
      return;
    std::vector<LandmarkCorrespondence> taintedElements;
    std::vector<LandmarkCorrespondenceIntMap::iterator> taintedElementsIterators;
    for (LandmarkCorrespondenceIntMap::iterator it=_landmarkCorrespondenceMap.begin();
	 it!=_landmarkCorrespondenceMap.end(); it++){
      const LandmarkCorrespondence& corr = it->first;
      if (corr.l1==lremoved || corr.l2==lremoved) {
	taintedElements.push_back(corr);
	taintedElementsIterators.push_back(it);
      }
    }
    // augment the number of occurrences of pattern l1, lx, when a pair l2-lx exists
    for (size_t i = 0; i<taintedElements.size(); i++){
      LandmarkCorrespondence& corr = taintedElements[i];
      LandmarkCorrespondenceIntMap::iterator it = taintedElementsIterators[i];
      int otherOccurrences = it->second;
      
      BaseTrackedLandmark* lOther = (corr.l1 == lremoved) ? corr.l2 : corr.l1;
      if (lOther == lkept)
	continue;
      
      addCorrespondence(lkept,lOther,otherOccurrences);
    }
    for (size_t i = 0; i<taintedElementsIterators.size(); i++) {
      LandmarkCorrespondenceIntMap::iterator it = taintedElementsIterators[i];
      _landmarkCorrespondenceMap.erase(it);
    }
  }

  int LandmarkCorrespondenceManager::merge(int minCount){
    int merged = 0;
    std::vector<LandmarkCorrespondence> mergedCorrespondences;
    do {
      mergedCorrespondences.clear();
      for (LandmarkCorrespondenceIntMap::iterator it=_landmarkCorrespondenceMap.begin();
	   it!=_landmarkCorrespondenceMap.end(); it++){
	if (it->second>minCount) {
	  mergedCorrespondences.push_back(it->first);
	  merged ++;
	}
      }
      for (size_t i = 0; i< mergedCorrespondences.size(); i++){
	LandmarkCorrespondence corr = mergedCorrespondences[i];
	BaseTrackedLandmark* l1=corr.l1;
	BaseTrackedLandmark* l2=corr.l2;
	mergeLandmarks(l1,l2);
	if (_mapperState->landmarks().count(l1) && _mapperState->landmarks().count(l2)){
	  _mapperState->mergeLandmarks(l1,l2);
	}
      }
    } while (mergedCorrespondences.size());
    return merged;
  }

}// end namespace
