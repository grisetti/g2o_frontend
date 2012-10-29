#include "g2o/stuff/timeutil.h"
#include "feature_tracker_pointxy.h"
#include <deque>
#include <assert.h>

namespace g2o {
  using namespace std;

  BaseTrackedLandmark* PointXYLandmarkConstructor::constructLandmark(std::vector<BaseTrackedFeature*>& v){
    if((int)v.size()<minNumObservations())
      return 0;
    BaseTrackedFeature* trackedFeature = v[0];
    FeaturePointXYData* featureData = trackedFeature->featureData<FeaturePointXYData*>();
    if (! featureData)
      return 0;

    VertexPointXY* landmarkVertex=dynamic_cast<VertexPointXY*>(featureData->constructLandmarkVertex());
    if (! landmarkVertex)
      return 0;
    VertexSE2* robotVertex=trackedFeature->frame()->vertex<VertexSE2*>();
    if (! robotVertex)
      return 0;
    // adjust the estimate
    landmarkVertex->setEstimate(robotVertex->estimate()*featureData->positionMeasurement());
    BaseTrackedLandmark* landmark = new BaseTrackedLandmark(landmarkVertex);
    return landmark;
  }

  OptimizableGraph::Edge* PointXYLandmarkConstructor::constructEdge(BaseTrackedLandmark* landmark, 
								 BaseTrackedFeature* trackedFeature){

    VertexPointXY* landmarkVertex=landmark->vertex<VertexPointXY*>();
    if (! landmarkVertex)
      return 0;

    FeaturePointXYData* featureData = trackedFeature->featureData<FeaturePointXYData*>();
    if (! featureData)
      return 0;

    VertexSE2* robotVertex=trackedFeature->frame()->vertex<VertexSE2*>();
    if (! robotVertex)
      return 0;

    // adjust the measurement
    EdgeSE2PointXY* e = dynamic_cast<EdgeSE2PointXY*>(featureData->constructEdge());
    if (! e)
      return 0;
    e->vertices()[0]=robotVertex;
    e->vertices()[1]=landmarkVertex;
    e->setMeasurement(featureData->positionMeasurement());
    e->setInformation(featureData->positionInformation());
    return e;
  }

  bool remapPose(Vector2d& newPose, Matchable* matchable, FeatureMappingMode mode){
    BaseTrackedFeature* trackedFeature = dynamic_cast<BaseTrackedFeature*>(matchable);
    if (trackedFeature) {
      FeaturePointXYData* featureData = trackedFeature->featureData<FeaturePointXYData*>();
      if (! featureData)
	return false;
      VertexSE2* robotVertex=trackedFeature->frame()->vertex<VertexSE2*>();
      BaseTrackedLandmark* landmark = trackedFeature->landmark();
      VertexPointXY* landmarkVertex = 0;
      if (landmark)
      landmarkVertex = trackedFeature->landmark()->vertex<VertexPointXY*>();
      if (! robotVertex)
	return false;
      switch (mode) {
      case LocalFrame:
	newPose = featureData->positionMeasurement();
	return true;
      case UseRobotPose: 
	newPose = robotVertex->estimate() * featureData->positionMeasurement();
	return true;
      case UseLandmark:
	if (! landmarkVertex)
	  return false;
	newPose = landmarkVertex->estimate();
	return true;
      case UseLandmarkIfAvailable:
	if (landmarkVertex)
	  newPose = landmarkVertex->estimate();
	else {
	  // cerr << "featureData" << featureData << "\t";
	  // cerr << "robotVertex" << robotVertex->id() << endl;
	  newPose = robotVertex->estimate() * featureData->positionMeasurement();
	}
	return true;
      }
      return false;
    }
    BaseTrackedLandmark* trackedLandmark = dynamic_cast<BaseTrackedLandmark*>(matchable);
    if (trackedLandmark) {
      if (mode == UseLandmark){
	VertexPointXY* landmarkVertex = trackedLandmark->vertex<VertexPointXY*>();
	newPose = landmarkVertex->estimate();
	return true;
      }
      return false;
    }
    return false;
  }

 
  PointXYInitialGuessCorrespondenceFinder::PointXYInitialGuessCorrespondenceFinder(){
    _featureMappingMode[0]=UseLandmarkIfAvailable;
    _featureMappingMode[1]=UseLandmarkIfAvailable;
    _squaredDistanceThreshold = 0.01;
  }

  void PointXYInitialGuessCorrespondenceFinder::compute(MatchableSet& s1, MatchableSet& s2){
    _correspondences.clear();
    if (!s1.size() || !s2.size())
      return;
    // compute the position of the features according to the frames to which they belong
    MatchablePoseMap fpmap;
    FeatureMappingMode  mode = _featureMappingMode[0];
    for (MatchableSet::iterator it= s1.begin(); it!=s2.end(); it++){
      if (it == s1.end()) {
	it = s2.begin();
	mode = _featureMappingMode[1];
      }
      Matchable* matchable = *it;
      Eigen::Vector2d remappedFeaturePose(0,0);
      bool remappingResult = remapPose(remappedFeaturePose,matchable,mode);
      if (!remappingResult){
	cerr << "FATAL!!!!!, no remapping result" << endl;
	return;
      }
      fpmap.insert(std::make_pair(matchable,remappedFeaturePose));
    }
    
    for (MatchableSet::iterator it1= s1.begin(); it1!=s1.end(); it1++){
      Matchable* trackedFeature1 = *it1;
      Vector2d pose1=fpmap[trackedFeature1];
      for (MatchableSet::iterator it2= s2.begin(); it2!=s2.end(); it2++) {
	Matchable* trackedFeature2 = *it2;
	Vector2d pose2=fpmap[trackedFeature2];
	double d = (pose1-pose2).squaredNorm();
	if (d<_squaredDistanceThreshold)
	  _correspondences.push_back(Correspondence(trackedFeature1, trackedFeature2, sqrt(d)));
      }
    }
    std::sort(_correspondences.begin(), _correspondences.end());
  }

  // void PointXYInitialGuessCorrespondenceFinder::compute(BaseTrackedFeatureSet& s1, BaseTrackedFeatureSet& s2){
  //   MatchableSet _s1;
  //   for (BaseTrackedFeatureSet::iterator it=s1.begin(); it!=s1.end(); it++){
  //     _s1.insert(*it);
  //   }
  //   MatchableSet _s2;
  //   for (BaseTrackedFeatureSet::iterator it=s2.begin(); it!=s2.end(); it++){
  //     _s2.insert(*it);
  //   }
  //   compute(_s1, _s2);
  // }

  // void PointXYInitialGuessCorrespondenceFinder::compute(BaseTrackedLandmarkSet& s1, BaseTrackedLandmarkSet& s2){
  //   MatchableSet _s1;
  //   for (BaseTrackedLandmarkSet::iterator it=s1.begin(); it!=s1.end(); it++){
  //     _s1.insert(*it);
  //   }
  //   MatchableSet _s2;
  //   for (BaseTrackedLandmarkSet::iterator it=s2.begin(); it!=s2.end(); it++){
  //     _s2.insert(*it);
  //   }
  //   compute(_s1, _s2);
  // }
  
  PointXYRansacMatcher::PointXYRansacMatcher() {
    _maxIterationsThreshold = 0.5;
    _minIterationsThreshold = 0.1;
    _inliersThreshold = 0.5;
    _squaredIntraFrameDistanceDifferenceThreshold = 1.;
    _squaredInterFrameDistanceDifferenceThreshold = 0.1;
    _squaredInlierDistanceThreshold = 0.1;
    _featureMappingMode[0]=UseLandmarkIfAvailable;
    _featureMappingMode[1]=UseLandmarkIfAvailable;
 }

  void PointXYRansacMatcher::compute(const CorrespondenceVector& correspondences_){
    
    // if we are matching landmarks, put in the beginning of the checks the landmarks that
    // are the same
    CorrespondenceVector correspondences = correspondences_;

    if (correspondences.size()<2){
      _matches = correspondences;
      _numInliers = correspondences.size();
      return;
    }

    int identityMatches = 0;
    if (_featureMappingMode[0] == UseLandmark && _featureMappingMode[1] == UseLandmark) {
      int k=0;
      for (size_t i = 0; i< correspondences.size(); i++) {
	Correspondence c = correspondences[i];
	BaseTrackedLandmark* l1=0, *l2=0;
	BaseTrackedFeature* f1 = dynamic_cast<BaseTrackedFeature*>(c.f1);
	BaseTrackedFeature* f2 = dynamic_cast<BaseTrackedFeature*>(c.f2);
	if (f1 && f2){
	  l1 = f1->landmark();
	  l2 = f2->landmark();
	} else {
	  l1 = dynamic_cast<BaseTrackedLandmark*>(c.f1);
	  l2 = dynamic_cast<BaseTrackedLandmark*>(c.f2);
	}
	if (! l1 || ! l2){
	  cerr << "fatal, no landmarks nor features in the pool" <<endl;
	  exit(0);
	}
	if (l1 == l2) {
	  Correspondence caux = correspondences[k];
	  correspondences[k] = c;
	  correspondences[i] = caux;
	  k++;
	}
      }
      identityMatches = k;
      std::sort(correspondences.begin()+k, correspondences.end());
    }
    
    _landmarkIdentityMatches = identityMatches;
    _numInliers = 0;
    _error = std::numeric_limits<double>::max();
    // first remap the correspondences
    _matches = correspondences;
    _transformation = SE2();
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > poses1(correspondences.size());
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > poses2(correspondences.size());

    
    for (size_t i=0; i<correspondences.size(); i++){
      const Correspondence& c = correspondences[i];
      Vector2d p1, p2;
      bool remappingResult = false;
      remappingResult = remapPose(p1, c.f1, _featureMappingMode[0]);
      if (! remappingResult){
	_matches.clear();
	return;
      }
      poses1[i] = p1;
      remappingResult = remapPose(p2, c.f2, _featureMappingMode[1]);
      if (! remappingResult) {
	_matches.clear();
	return;
      }
      poses2[i] = p2;
      _matches[i].distance = std::numeric_limits<double>::max();
    }
    
    int totalMaxIterations = (correspondences.size() * (correspondences.size()-1))/2;
    if (identityMatches > 2) 
      totalMaxIterations = (identityMatches * (identityMatches -1) /2);
    //cerr << "totalMaxIterations" << totalMaxIterations << endl;
    int maxIterations = (int) (_maxIterationsThreshold * totalMaxIterations);
    int minIterations = (int) (_minIterationsThreshold * totalMaxIterations);
    int numInliersStop = (int) (_inliersThreshold * correspondences.size());

    SE2 bestTransformation;
    int bestInliers = -1;
    double bestError = std::numeric_limits<double>::max();
    std::vector<double> bestErrors(correspondences.size());
    std::vector<double> currentErrors(correspondences.size());

    //cerr << "maxIterations=" << maxIterations << endl;
    int numIterations = 0;
    // now sample pairs of correspondences, in a systematic manner
    for (size_t i = 0; i<correspondences.size()-1; i++){
      const Correspondence& c1 = correspondences[i];
      Vector2d& p11 = poses1[i];
      Vector2d& p12 = poses2[i];
      for (size_t j = i+1; j<correspondences.size(); j++){
	numIterations ++;
	if (numIterations >= maxIterations ){
	  std::sort(_matches.begin(), _matches.end());
	  _matches.resize(_numInliers);
	  return;
	}

	const Correspondence& c2 = correspondences[j];
	if (c1.f1==c2.f1 || c1.f2 == c2.f2)
	  continue;
	Vector2d& p21 = poses1[j];
	Vector2d& p22 = poses2[j];
	
	Vector2d delta1 = p21 - p11;
	Vector2d delta2 = p22 - p12;
	
	double dA=delta1.squaredNorm();
	double dB=delta2.squaredNorm();

	// cerr << "dA= " << dA << " dB=" << dB << endl;
	// cerr << "fabs(dA - dB)= " << fabs(dA - dB) << endl;

	// reject for the matching pairs of features that are too close,
	// as they are unstable
	if (dA<_squaredInterFrameDistanceDifferenceThreshold)
	  continue;
	if (dB<_squaredInterFrameDistanceDifferenceThreshold)
	  continue;

	
	// reject the match if the euclidean distance calculated in the different frames differ too much
	// as they are likely to be wrong1
	if (fabs(dA - dB)> _squaredIntraFrameDistanceDifferenceThreshold)
	  continue;
	
	// compute a transform that aligns the frames based on the correspondences 
	double theta1 = atan2 ( delta1.y(), delta1.x());
	double theta2 = atan2 ( delta2.y(), delta2.x());
	double deltaTheta = theta1-theta2;
	double c=cos(deltaTheta);
	double s=sin(deltaTheta);

	//cerr << "deltaTheta= " << deltaTheta << endl;

	Vector2d mean1 = (p21 + p11) * .5;
	Vector2d mean2 = (p22 + p12) * .5;

	Eigen::Matrix2d R;
	R << c ,-s , s , c;
	Vector2d deltaT = mean1 - R * mean2;
	
	// the transformation is in deltaTheta and deltaT;
	// apply the transformation to all points in the second set;
	
	int currentInliers = 0;
	double currentError = 0;
	for (size_t k = 0; k<correspondences.size(); k++){
	  Vector2d pt = deltaT + R * poses2[k];
	  double dist=(poses1[k] - pt).squaredNorm();
	  if (dist < _squaredInlierDistanceThreshold){
	    currentInliers ++;
	    currentErrors[k] = dist;
	    currentError += dist;
	  } else
	    currentErrors[k] = std::numeric_limits<double>::max();
	}

	if (currentInliers >= bestInliers && currentError < bestError){
	  bestErrors = currentErrors;
	  for (size_t k = 0; k<correspondences.size(); k++){
	    _matches[k].distance = currentErrors[k];
	  }
	  bestError = currentError;
	  bestInliers = currentInliers;
	  bestTransformation = SE2(deltaT.x(), deltaT.y(), deltaTheta);
	  _transformation  = bestTransformation;
	  _numInliers = currentInliers;
	}
	
	if (numIterations > minIterations && bestInliers >= numInliersStop){
	  std::sort(_matches.begin(), _matches.end());
	  _matches.resize(_numInliers);
	  return;
	}
      }
    }
  }

 
  
  SE2LoopClosureCandidateDetector::SE2LoopClosureCandidateDetector(MapperState* mapperState_)
    :LoopClosureCandidateDetector(mapperState_){
    _squaredLinearClosureThreshold = 25;
    _angularClosureThreshold = M_PI;
  }
  
  
  void SE2LoopClosureCandidateDetector::compute(BaseFrame* frame) {
    _candidates.clear();
    VertexSE2* v = frame->vertex<VertexSE2*>();
 
    // do an ugly greedy search for frames which are metrically near to the actual one;
    assert(v);
    SE2 estimate = v->estimate();
    cerr << "num closing candidates for vertex " << v->id() << ": ";
    for (VertexFrameMap::iterator it=_mapperState->frames().begin(); it!=_mapperState->frames().end(); it++){
      BaseFrame* f = it->second;
      VertexSE2* v2 = f->vertex<VertexSE2*>();
      assert(v2);
      //cerr << "check " << v2->id() << endl;
      if (v == v2)
	continue;
      SE2 estimate2 = v2->estimate();
      SE2 delta =  estimate.inverse() * estimate2;
      if (delta.translation().squaredNorm()<_squaredLinearClosureThreshold &&
	  fabs(delta.rotation().angle())<_angularClosureThreshold){
	_candidates.insert(f);
	//cerr << v2->id() << " ";
      }
    }
    cerr << _candidates.size() << endl;
  }
  

  bool PointXYLandmarkDistanceEstimator::compute(double& distance, BaseTrackedLandmark* l1, BaseTrackedLandmark* l2){
    VertexPointXY* v1 = l1->vertex<VertexPointXY*>();
    VertexPointXY* v2 = l2->vertex<VertexPointXY*>();
    if (! v1 || ! v2)
      return false;
    distance = (v1->estimate()-v2->estimate()).norm();
    return true;
  }
}// end namespace
