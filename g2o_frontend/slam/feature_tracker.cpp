#include "feature_tracker.h"
#include <assert.h>
#include "g2o/core/sparse_optimizer.h"

namespace g2o {
  using namespace std;

  BaseFrame::BaseFrame(OptimizableGraph::Vertex* v,
		       OptimizableGraph::Edge* e,
		       BaseFrame* p,
		       BaseFrame* n):
    _vertex(v), _odometryEdge(e), _previousFrame(p), _nextFrame(n) {
  }

  void BaseFrame::setVertex(OptimizableGraph::Vertex* v) { _vertex = v; }


  void BaseFrame::landmarks(BaseTrackedLandmarkSet& landmarks_){
    for (BaseTrackedFeatureSet::iterator it = _features.begin(); it!=_features.end(); it++){
      BaseTrackedFeature* f = *it;
      BaseTrackedLandmark* l = f->landmark();
      if (l){
	landmarks_.insert(l);
      }
    }
  }
  
  BaseTrackedFeature::BaseTrackedFeature(BaseFrame* frame_, 
					 BaseFeatureData* featureData_, 
					 BaseTrackedFeature* previous_):
    _frame(frame_), _featureData(featureData_), _previous(previous_) {
    _landmark = 0;
    _edge = 0;
    setPrevious(previous_);
  }

  BaseTrackedFeature::~BaseTrackedFeature(){}
  
  int BaseTrackedFeature::trackLenght() const {
    const BaseTrackedFeature* f=this;
    int l=1;
    while(f->previous()){
      f=f->previous();
      l++;
    }
    return l;
  }

  void BaseTrackedFeature::setPrevious(BaseTrackedFeature* previous_) {
    if (_previous != 0){
      _previous->_children.erase(this);
    }
    if (previous_ != 0){
      previous_->_children.insert(this);
    }
    _previous = previous_;
  }

  

  BaseTrackedLandmark::BaseTrackedLandmark(OptimizableGraph::Vertex* v)
    :_vertex(v) {}
  
  Matcher::Matcher(){
    _landmarkIdentityMatches = 0;
  }
  
  bool Matcher::applyCandidateTransform(){
    return false;
  }

  CorrespondenceSecondMatchFilter::CorrespondenceSecondMatchFilter(
								   double distanceDifferenceThreshold_, 
								   double distanceRatioThreshold_): 
    _distanceDifferenceThreshold(distanceDifferenceThreshold_),
    _distanceRatioThreshold(distanceRatioThreshold_) { }

  struct CorrespondenceIdSorter {
    CorrespondenceIdSorter() {
      _order = true;
    }
    inline void setOrder(bool order_) { _order = order_; }
    inline bool order() const {return _order;}

    bool operator()(const Correspondence& c1, const Correspondence& c2) const {
      if (_order) {
	return (c1.f1<c2.f1 || (c1.f1==c2.f1 && c1.distance < c2.distance) );
      } else {
	return (c1.f2<c2.f2 || (c1.f2==c2.f2 && c1.distance < c2.distance) );
      }
    } 
    bool _order;
  };

  void CorrespondenceSecondMatchFilter::compute(const CorrespondenceVector& correspondences){
    _filtered.clear();
    if (correspondences.empty())
      return;
    _filtered = correspondences;
    
    std::set<int> tainted;
    for (int k=0; k<2; k++) {
      CorrespondenceIdSorter cs;
      cs.setOrder(k==0);
      sort(_filtered.begin(), _filtered.end(), cs);
    
      int prevIdx = -1;
     BaseTrackedFeature* prev = 0;
      double prevDistance = std::numeric_limits<double>::max();
      
      bool firstFound = false;
      for (size_t i = 0; i<_filtered.size(); i++){
	BaseTrackedFeature* f=(k==0)? _filtered[i].f1 : _filtered[i].f2;
	double distance = _filtered[i].distance;
	//cerr << "k: " << k << " f1:" << _filtered[i].f1 << " f2:" << _filtered[i].f2 << " d:" <<  distance <<endl;
	if (prev!=f) {
	  prevIdx = i;
	  prev=f;
	  prevDistance = distance;
	  firstFound = true;
	  continue;
	}  else {
	  double dDelta=distance-prevDistance;
	  if (firstFound ){
	    if (dDelta < _distanceDifferenceThreshold){
	      tainted.insert(prevIdx);
	    } 
	  } 
	  tainted.insert(i);
	  firstFound = false;
	}
      }
    }
    for (std::set<int>::iterator it=tainted.begin(); it!=tainted.end(); it++){
      int i=*it;
      _filtered[i].distance=std::numeric_limits<double>::max();
    }
    std::sort(_filtered.begin(), _filtered.end());
    _filtered.resize(_filtered.size()-tainted.size());

  }


  MapperState::MapperState(OptimizableGraph* graph_, 
				 LandmarkConstructor* landmarkConstructor_){
    _graph = graph_;
    _landmarkConstructor = landmarkConstructor_;
    _lastFrame =0;
    _minLandmarkCreationFrames = 2;
    _minLandmarkCommitFrames = 3;
    _runningLandmarkId = 1000000;
  }
  
  bool MapperState::setNeighborFrames(BaseFrame* f1, BaseFrame* f2, bool connect_){
    if (connect_) {
      if(f1->neighbors().count(f2))
	return false;
      f1->neighbors().insert(f2);
      f2->neighbors().insert(f1);
    } else {
      //if (f1->previous()==f2 || f2->previous()==f1) return false;
      if(!f1->neighbors().count(f2))
	return false;
      f1->neighbors().erase(f2);
      f2->neighbors().erase(f1);
    }
    return true;
  }
  

  // this adds a landmark to the pool, not to the graph
  void MapperState::addLandmark(BaseTrackedLandmark* l) {
    _landmarks.insert(l);
    if (l->vertex<OptimizableGraph::Vertex*>())
      _graph->addVertex(l->vertex<OptimizableGraph::Vertex*>());
  }

  // this removes a landmark from the pool and from all tracked features that were referencing it;
  void MapperState::removeLandmark(BaseTrackedLandmark* l){
    //erase the landmark from all features where the landmark was observed;
    for (BaseTrackedFeatureSet::iterator it = l->features().begin();
	 it!=l->features().end(); it++){
      BaseTrackedFeature* f=*it;
      f->setLandmark(0);
    }
    // erase the vertex
    OptimizableGraph::Vertex* v=l->vertex<OptimizableGraph::Vertex*>();
    if (v)
      _graph->removeVertex(v);
    _landmarks.erase(l);
    delete l;
  }

  void MapperState::mergeLandmarks(BaseTrackedLandmark* l1, BaseTrackedLandmark* l2) {
    if (!l1 || !l2)
      return;
    if (l1==l2)
      return;
    //cerr << "merging landmarks: " << l1 << " " << l2 << endl;
    OptimizableGraph::Vertex* v1 = l1->vertex<OptimizableGraph::Vertex*>();
    OptimizableGraph::Vertex* v2 = l2->vertex<OptimizableGraph::Vertex*>();
    for (BaseTrackedFeatureSet::iterator it=l2->features().begin(); it!=l2->features().end(); it++){
      BaseTrackedFeature* feature=*it;
      //cerr << "\tfeature: " << feature << endl;
      OptimizableGraph::Edge* edge = feature->edge<OptimizableGraph::Edge*>();
      assert(feature->landmark() == l2);
      v2->edges().erase(edge);
      for (size_t i =0 ; i<edge->vertices().size(); i++){
	if (edge->vertices()[i]==v2)
	  edge->vertices()[i]=v1;
      }
      v1->edges().insert(edge);
      feature->setEdge(edge);
      feature->setLandmark(l1);
      l1->features().insert(feature);
      assert(feature->landmark() == l1);
    }
    _graph->removeVertex(v2);
    _landmarks.erase(l2);
    delete l2;
  }

  // this adds a landmark and all edges to the graph
  void MapperState::confirmLandmark(BaseTrackedLandmark* /*f*/){
  }

  // this adds e new frame at the end of the history
  void MapperState::addFrame(OptimizableGraph::Vertex* v, OptimizableGraph::Edge* odometry) {
    BaseFrame * frame = new BaseFrame(v,odometry,_lastFrame, 0);
    if (_lastFrame){
      _lastFrame->setNext(frame);
      _lastFrame->neighbors().insert(frame);
      //frame->neighbors().insert(_lastFrame);
    }
    _lastFrame = frame;
    _frames.insert(std::make_pair(v,frame));
  }

  void MapperState::addTrackedFeature(BaseTrackedFeature* feature) {
    BaseFrame * frame = feature->frame();
    frame->features().insert(feature);
    // consistency check;
    // if (feature->previous())
    //   assert("addTrackedFeature:  ERROR, previous frame does not match previous feature frame" && feature->previous()->frame() == frame->previous());
    BaseTrackedLandmark* landmark = feature->landmark();
    if (landmark && !_landmarks.count(landmark)){
      assert (0 && "addTrackedFeature: ERROR, landmark must be already inserted in the pool");
    }
  }

  // this removes a track of features from the pool
  // if they reference a landmark, also the landmark is removed
  int MapperState::removeTrackedFeature(BaseTrackedFeature* feature, bool recursive, bool removeLandmarks){
    // detach the next features
    BaseTrackedFeatureSet children = feature->children();
    for (BaseTrackedFeatureSet::iterator it=children.begin(); it!=children.end(); it++){
      BaseTrackedFeature* f = *it;
      f->setPrevious(0);
    }

    // if there is a landmark and the landmark has no observations after matching the feature, delete the landmark
    BaseTrackedLandmark * l = feature->landmark();
    if (removeLandmarks && l ) {
      OptimizableGraph::Edge* e = feature->edge<OptimizableGraph::Edge*>();
      if (e)
	_graph->removeEdge(e);
      l->features().erase(feature);
      if (l->features().empty()) 
	removeLandmark(feature->landmark());
    }

    // if the shit is recursive go backwards and remove all the track
    BaseTrackedFeature* previousFeature = feature->previous();
    if (feature->frame())
      feature->frame()->features().erase(feature);
    delete feature;
    return 1;
    if (recursive && previousFeature && previousFeature->numChildren()==0) {
      return 1 + removeTrackedFeature(previousFeature, recursive, removeLandmarks);
    }
    return 0;
  }


  void MapperState::updateTracksAndLandmarks(const CorrespondenceVector& correspondences) {
    cerr << "Updating Tracks" << endl;
    for (size_t i=0; i<correspondences.size(); i++){
      Correspondence  c = correspondences[i];
      BaseTrackedFeature* f1 = c.f1;
      BaseTrackedFeature* f2 = c.f2;
      f2->setPrevious(f1);
    }
  
    for (size_t i =0; i<correspondences.size(); i++){
      Correspondence  c = correspondences[i];
      BaseTrackedFeature* f1 = c.f1;
      BaseTrackedFeature* f2 = c.f2;

      int l=f2->trackLenght();

      // cerr << "\t\ti:" << i << " " 
      // 	   << f1 << "," << f2 
      // 	   << " d:" << c.distance << " l:" << l << endl;
      
      if (l>=_minLandmarkCreationFrames && l>=_landmarkConstructor->minNumObservations()){
	BaseTrackedLandmark* landmark = f1->landmark();
	if (! landmark){
	  // pack a vector of observations that holds a sequence sufficient to initialize the landmark
	  std::vector<BaseTrackedFeature*> features(l);
	  BaseTrackedFeature*f=f2;
	  int k=0;
	  while(f){
	    features[k++]=f;
	    f=f->previous();
	  }
	  landmark = _landmarkConstructor->constructLandmark(features);
	  assert (landmark);
	  landmark->vertex<OptimizableGraph::Vertex*>()->setId(_runningLandmarkId++);
	  addLandmark(landmark);
	  
	  // assign the same landmark to all features in the track, and construct a meaningful observation
	  f=f2;
	  while(f) {
	    f->setLandmark(landmark);
	    OptimizableGraph::Edge* e = _landmarkConstructor->constructEdge(landmark, f);
	    assert(e);
	    f->setEdge(e);
	    landmark->features().insert(f);
	    f=f->previous();
	    bool retval = _graph->addEdge(e);
	    if (! retval) {
	      assert(0 && "error creating edge");
	    }
	  }
	  //cerr << "made new landmark, id:" << landmark->vertex<OptimizableGraph::Vertex*>()->id() << endl;

	} else {
	  //cerr << "extending existant landmark, id:" << landmark->vertex<OptimizableGraph::Vertex*>()->id() << endl;
	  f2->setLandmark(landmark);
	  OptimizableGraph::Edge* e = _landmarkConstructor->constructEdge(landmark, f2);
	  f2->setEdge(e);
	  bool retval = _graph->addEdge(e);
	  if (! retval) {
	    assert(0 && "error creating edge");
	  }
	  landmark->features().insert(f2);
	}
      }
    }
  }

  
  void MapperState::commonLandmarks(BaseTrackedLandmarkSet& common, BaseFrame* f1, BaseFrame* f2) {
    common.clear();
    assert(f1!=f2);
    BaseTrackedLandmarkSet landmarks1, landmarks2;
    f1->landmarks(landmarks1);
    f2->landmarks(landmarks2);
    
    std::set_intersection(landmarks1.begin(), landmarks1.end(), 
			  landmarks2.begin(), landmarks2.end(),
			  inserter(common, common.begin()));
  }

  
  typedef std::set<BaseTrackedLandmark*> BaseLandmarkSet;
  
  void MapperState::selectFeaturesWithLandmarks(BaseTrackedFeatureSet& featureSet, BaseFrameSet& frameSet){
    featureSet.clear();
    BaseLandmarkSet landmarks;
    for (BaseFrameSet::iterator it=frameSet.begin(); it!=frameSet.end(); it++){
      BaseFrame* frame = *it;
      for (BaseTrackedFeatureSet::iterator fit=frame->features().begin(); 
	   fit!=frame->features().end(); fit++){
	BaseTrackedFeature* feature = *fit;
	BaseTrackedLandmark* landmark=feature->landmark();
	if (landmark && !landmarks.count(feature->landmark())){
	    landmarks.insert(feature->landmark());
	    featureSet.insert(feature);
	}
      }
    }
  }

  void MapperState::selectFeaturesWithLandmarks(BaseTrackedFeatureSet& featureSet, BaseFrame* frame){
    BaseFrameSet fset;
    fset.insert(frame);
    selectFeaturesWithLandmarks(featureSet, fset);
  }
  

  BaseFrame* MapperState::lastNFrames(BaseFrameSet& fset, int nFramesBack) {
    fset.clear();
    BaseFrame* f=lastFrame();
    BaseFrame* fp=lastFrame();
    while (f && nFramesBack>0){
      nFramesBack --;
      fset.insert(f);
      fp =f;
      f=f->previous();
    }
    if (! fp){
      fset.clear();
    }
    return fp;
  }


  int MapperState::refineConnectivity(BaseFrameSet& frames, int minCommonLandmarks, bool odometryIsGood){
    int addedLinks = 0;
  for (BaseFrameSet::iterator it = frames.begin(); it!=frames.end(); it++){
    BaseFrameSet::iterator it2=it; it2++;
    BaseFrame* f1 = *it;
    for (; it2!=frames.end(); it2++){
      BaseFrame* f2 = *it2;
      if (f1==f2)
	continue;
      BaseTrackedLandmarkSet commonSet;
      commonLandmarks(commonSet, f1, f2 );
      if ((int)commonSet.size()>minCommonLandmarks ||
	  (odometryIsGood && (f1->previous() == f2 || f2->previous() == f1)) ){
	bool result = setNeighborFrames(f1, f2, true);
	if (result)
	  addedLinks ++;
      }
    }
  }
  return addedLinks;
}
  

  Tracker::Tracker(MapperState* mapperState_, CorrespondenceFinder* correspondenceFinder_, Matcher* matcher_) {
      _mapperState = mapperState_;
      _correspondenceFinder = correspondenceFinder_;
      _matcher = matcher_;
  }

  
  void Tracker::searchInitialCorrespondences(CorrespondenceVector& correspondences_, int numFrames) {
    assert (_correspondenceFinder);
    correspondences_.clear();
    BaseFrame* current = _mapperState->lastFrame();
    if (! current)
      return;
    BaseFrame* previous = current->previous();
    BaseTrackedFeatureSet openFeatures=current->features();
    int k = 1;
    cerr << "start" << endl;
    while (previous && numFrames >0 && ! openFeatures.empty()) {
      cerr << "f: " << k << " ptr:" << previous << " #features:" << previous->features().size() << endl; 
      k++;
      _correspondenceFinder->compute(previous->features(),openFeatures);
      const CorrespondenceVector& correspondences = _correspondenceFinder->correspondences();
      cerr << "Odom: found " << correspondences.size() << " correspondences" << endl;
      /*
	for (size_t i =0; i<correspondences.size(); i++){
	cerr << "\t\t" << i << " " 
	<< correspondences[i].f1 << "," << correspondences[i].f2 
	<< " d=" << correspondences[i].distance << endl;
	}
      */
    
      assert(_matcher);
      _matcher->compute(correspondences);
      const CorrespondenceVector& matches = _matcher->matches();
     
      cerr << "RANSAC " << matches.size() << " matches" << endl;

      for (size_t i = 0; i<matches.size(); i++){
	const Correspondence& c = matches[i];
	openFeatures.erase(c.f2);
      }
      correspondences_.insert(correspondences_.end(), matches.begin(), matches.end());
      previous = previous->previous();
      numFrames --;
    }
    cerr << "end" << endl;
    
  }

}// end namespace
