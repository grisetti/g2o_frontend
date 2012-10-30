#include "feature_tracker.h"
#include <assert.h>
#include "g2o/core/sparse_optimizer.h"

namespace g2o {
  using namespace std;

  BaseFrame::BaseFrame(OptimizableGraph::Vertex* v):
    _vertex(v) {
  }

  void BaseFrame::setVertex(OptimizableGraph::Vertex* v) { _vertex = v; }

  void BaseFrame::landmarks(MatchableSet& landmarks_){
    for (MatchableSet::iterator it = _matchables.begin(); it!=_matchables.end(); it++){
      BaseTrackedLandmark* l = 0;
      BaseTrackedFeature* f = dynamic_cast<BaseTrackedFeature*>(*it);
      if (f) {
	l=f->landmark();
      } else {
	l = dynamic_cast<BaseTrackedLandmark*>(*it);
      }
      if (l) {
	landmarks_.insert(l);
      }
    }
  }

  void BaseFrame::features(MatchableSet& features_){
    for (MatchableSet::iterator it = _matchables.begin(); it!=_matchables.end(); it++){
      BaseTrackedFeature* f = dynamic_cast<BaseTrackedFeature*>(*it);
      if (f) {
	features_.insert(f);
      }
    }
  }

  BaseSequentialFrame::BaseSequentialFrame(OptimizableGraph::Vertex* v,
		       OptimizableGraph::Edge* e,
		       BaseSequentialFrame* p,
		       BaseSequentialFrame* n):
    BaseFrame(v), _odometryEdge(e), _previousFrame(p), _nextFrame(n) {
  }

  Matchable::~Matchable(){}

  BaseTrackedFeature::BaseTrackedFeature(BaseSequentialFrame* frame_, 
					 BaseFeatureData* featureData_, 
					 BaseTrackedFeature* previous_):
    _frame(frame_), _featureData(featureData_), _previous(previous_) {
    _landmark = 0;
    _edge = 0;
    setPrevious(previous_);
  }

  BaseTrackedFeature::~BaseTrackedFeature(){
    //cerr << "\t\tdeleting feature " << this << endl;
    //assert (0);
  }
  
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
     Matchable* prev = 0;
      double prevDistance = std::numeric_limits<double>::max();
      
      bool firstFound = false;
      for (size_t i = 0; i<_filtered.size(); i++){
	Matchable* f=(k==0)? _filtered[i].f1 : _filtered[i].f2;
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
  
  bool MapperState::setNeighborFrames(BaseSequentialFrame* f1, BaseSequentialFrame* f2, bool connect_){
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
  }    std::set<BaseTrackedLandmark*> _landmarks;
    VertexFrameMap _frames;


  // this adds a landmark and all edges to the graph
  void MapperState::confirmLandmark(BaseTrackedLandmark* /*f*/){
  }

  // this adds e new frame at the end of the history
  void MapperState::addFrame(OptimizableGraph::Vertex* v, OptimizableGraph::Edge* odometry) {
    BaseSequentialFrame * frame = new BaseSequentialFrame(v,odometry,_lastFrame, 0);
    if (_lastFrame){
      _lastFrame->setNext(frame);
      //_lastFrame->neighbors().insert(frame);
      //frame->neighbors().insert(_lastFrame);
    }
    _lastFrame = frame;
    _frames.insert(std::make_pair(v,frame));
  }

  void MapperState::addTrackedFeature(BaseTrackedFeature* feature) {
    BaseSequentialFrame * frame = feature->frame();
    frame->matchables().insert(feature);
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
  int MapperState::removeTrackedFeature(BaseTrackedFeature* feature, bool recursive){
    // remove the feature from the frame
    assert(feature->frame());
    if (feature->frame())
      feature->frame()->matchables().erase(feature);
    // detach the next features
    BaseTrackedFeatureSet children = feature->children();
    for (BaseTrackedFeatureSet::iterator it=children.begin(); it!=children.end(); it++){
      BaseTrackedFeature* f = *it;
      f->setPrevious(0);
    }
    
    // if there is a landmark and the landmark has no observations after matching the feature, delete the landmark
    BaseTrackedLandmark * l = feature->landmark();
    if (l){
      OptimizableGraph::Edge* e = feature->edge<OptimizableGraph::Edge*>();
      if (e)
	_graph->removeEdge(e);
      l->features().erase(feature);
      if (l->features().empty()) 
	removeLandmark(l);
    }

    // if the shit is recursive go backwards and remove all the track
    BaseTrackedFeature* previousFeature = feature->previous();
    delete feature;
    if (recursive && previousFeature && previousFeature->numChildren()==0) {
      return 1 + removeTrackedFeature(previousFeature, recursive);
    }
    return 1;
  }


  void MapperState::updateTracksAndLandmarks(const CorrespondenceVector& correspondences_){
    CorrespondenceVector correspondences(correspondences_.size());
    int k=0;
    for (size_t i=0; i<correspondences_.size(); i++){
      Correspondence  c = correspondences_[i];
      BaseTrackedFeature* f1 = dynamic_cast<BaseTrackedFeature*>(c.f1);
      BaseTrackedFeature* f2 = dynamic_cast<BaseTrackedFeature*>(c.f2);
      if (f1 && f2) {
	correspondences[k]=c;
	k++;
      }
    }
    correspondences.resize(k);

    cerr << "Updating Tracks" << endl;
    for (size_t i=0; i<correspondences.size(); i++){
      Correspondence  c = correspondences[i];
      BaseTrackedFeature* f1 = static_cast<BaseTrackedFeature*>(c.f1);
      BaseTrackedFeature* f2 = static_cast<BaseTrackedFeature*>(c.f2);
      f2->setPrevious(f1);
    }
  
    for (size_t i =0; i<correspondences.size(); i++){
      Correspondence  c = correspondences[i];
      BaseTrackedFeature* f1 = static_cast<BaseTrackedFeature*>(c.f1);
      BaseTrackedFeature* f2 = static_cast<BaseTrackedFeature*>(c.f2);

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
	  // cerr << "made new landmark, id:" << landmark->vertex<OptimizableGraph::Vertex*>()->id() << endl;

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

  
  void MapperState::commonLandmarks(MatchableSet& common, BaseSequentialFrame* f1, BaseSequentialFrame* f2) {
    common.clear();
    assert(f1!=f2);
    MatchableSet landmarks1, landmarks2;
    f1->landmarks(landmarks1);
    f2->landmarks(landmarks2);
    
    std::set_intersection(landmarks1.begin(), landmarks1.end(), 
			  landmarks2.begin(), landmarks2.end(),
			  inserter(common, common.begin()));
  }

  
  typedef std::set<BaseTrackedLandmark*> BaseLandmarkSet;
  
  
  void MapperState::selectLandmarks(MatchableSet& landmarks, BaseFrameSet& frameSet){
    landmarks.clear();
    for (BaseFrameSet::iterator it=frameSet.begin(); it!=frameSet.end(); it++){
      BaseSequentialFrame* frame = *it;
      frame->landmarks(landmarks);
    }
  }
 
  BaseSequentialFrame* MapperState::lastNFrames(BaseFrameSet& fset, int nFramesBack) {
    fset.clear();
    BaseSequentialFrame* f=lastFrame();
    BaseSequentialFrame* fp=lastFrame();
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
    BaseSequentialFrame* f1 = *it;
    for (; it2!=frames.end(); it2++){
      BaseSequentialFrame* f2 = *it2;
      if (f1==f2)
	continue;
      MatchableSet commonSet;
      commonLandmarks(commonSet, f1, f2 );
      int numCommonLandmarks = commonSet.size();
      bool framesAreConnected = (f1->neighbors().count(f2) && f2->neighbors().count(f1));
      bool framesAreConsecutive = (f1->previous() == f2) || (f2->previous() == f1);
      if (numCommonLandmarks > minCommonLandmarks ||
	  (odometryIsGood && framesAreConsecutive) ){
	bool result = setNeighborFrames(f1, f2, true);
	if (result)
	  addedLinks ++;
      } else {
	if (framesAreConnected){
	  bool result = setNeighborFrames(f1, f2, false);
	  if (result)
	    addedLinks --;
	}
      }
    }
  }
  return addedLinks;
}

  MapperState::~MapperState() {

   cerr << "deleting landmarks" << endl;
     for (std::set<BaseTrackedLandmark*>::iterator it=_landmarks.begin(); it!=_landmarks.end(); it++){
      BaseTrackedLandmark* l=*it;
      delete l;
    }
    cerr << "deleting features" << endl;
    for (VertexFrameMap::iterator it = _frames.begin(); it!=_frames.end(); it++){
      // cerr << "\t frame:" << it->first->id() << endl;
	BaseSequentialFrame* frame = it->second;
	MatchableSet features;
	frame->features(features);
	for (MatchableSet::iterator fit = features.begin(); fit !=features.end(); fit++){
	    delete *fit;
	}
    }
    //delete frame;
    cerr << "done" << endl;
  }


  Tracker::Tracker(MapperState* mapperState_, CorrespondenceFinder* correspondenceFinder_, Matcher* matcher_) {
      _mapperState = mapperState_;
      _correspondenceFinder = correspondenceFinder_;
      _matcher = matcher_;
  }

  
  void Tracker::searchInitialCorrespondences(CorrespondenceVector& correspondences_, int numFrames) {
    assert (_correspondenceFinder);
    correspondences_.clear();
    BaseSequentialFrame* current = _mapperState->lastFrame();
    if (! current)
      return;
    BaseSequentialFrame* previous = current->previous();
    MatchableSet openFeatures=current->matchables();
    int k = 1;
    cerr << "start" << endl;
    MatchableSet currentFeatures;
    current->features(currentFeatures);
    while (previous && numFrames >0 && ! openFeatures.empty()) {
      MatchableSet previousFeatures;
      previous->features(previousFeatures);
      cerr << "f: " << k << " ptr:" << previous << " #features:" << previousFeatures.size() << endl; 
      k++;
      _correspondenceFinder->compute(previous->matchables(),openFeatures);
      const CorrespondenceVector& correspondences = _correspondenceFinder->correspondences();
      cerr << "Odom: found " << correspondences.size() << " correspondences" << endl;
      /*
	for (size_t i =0; i<correspondences.size(); i++){
	cerr << "\t\t" << i << " " 
	<< correspondences[i].f1 << "," << correspondences[i].f2 
	<< " d=" << correspondences[i].distance << endl;
	}
      */
      // if (correspondences.size()==1 && correspondences[0].distance >.3)
      // 	correspondences.clear();

      assert(_matcher);
      _matcher->compute(correspondences);
      const CorrespondenceVector& matches = _matcher->matches();
     
      cerr << "RANSAC " << matches.size() << " matches" << endl;

      for (size_t i = 0; i<matches.size(); i++){
	const Correspondence& c = matches[i];
	openFeatures.erase(static_cast<BaseTrackedFeature*>(c.f2));
      }
      correspondences_.insert(correspondences_.end(), matches.begin(), matches.end());
      previous = previous->previous();
      numFrames --;
    }

    cerr << "end" << endl;
    
  }

}// end namespace
