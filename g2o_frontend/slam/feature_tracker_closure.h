#ifndef G2O_FRONTEND_FEATURE_TRACKER_CLOSURE
#define G2O_FRONTEND_FEATURE_TRACKER_CLOSURE
#include "feature_tracker.h"

namespace g2o {

  /**class that computes the candidate frames in the pool where to look for a loop closure
   */
  struct LoopClosureCandidateDetector{
		
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    LoopClosureCandidateDetector (FeatureTracker* tracker_);
    virtual void compute(BaseFrame* current) = 0;
    inline FeatureTracker* tracker() { return _tracker; }
    inline const FeatureTracker* tracker() const {return _tracker;}
    inline OptimizableGraph* graph() {return _tracker->graph();}
    inline const OptimizableGraph* graph() const {return _tracker->graph();}
    inline const BaseFrameSet& candidates() const { return _candidates; };
    inline BaseFrameSet& candidates() {return _candidates; }

  protected:
    FeatureTracker* _tracker;
    BaseFrameSet _candidates;
  };
  
  /**Selects a set of edges/vertices from a set of frames.
     Used to extract a portion of the optimization graph, starting from a set of frames.
   */
  struct GraphItemSelector {
		
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    GraphItemSelector();
    virtual void compute(BaseFrameSet& frameSet);
    inline HyperGraph::EdgeSet& selectedEdges() {return _selectedEdges;}
    inline OptimizableGraph::VertexSet& selectedVertices() {return _selectedVertices;}
  protected:
    HyperGraph::EdgeSet _selectedEdges;
    OptimizableGraph::VertexSet _selectedVertices;
  };


  /**Determines the connected regions in a set of frames.
   */
  struct FrameClusterer {
		
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void compute(BaseFrameSet& frameSet);
    inline BaseFrameSet& cluster(int i) {return _clusters.at(i);}
    inline int numClusters() const {return _clusters.size(); }
  protected:
    std::vector<BaseFrameSet> _clusters;
  };
  

  /**Correspondence, but for landmarks. Used when merging local maps
   */
  struct LandmarkCorrespondence {
		
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    LandmarkCorrespondence(BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_);

    inline bool operator < (const LandmarkCorrespondence& c) const 
    { return l1 < c.l1 || ((l1 == c.l1) && (l2 < c.l2)); }

    BaseTrackedLandmark *l1, *l2;  
  };


  /**Struct that keeps track of  "Counting" how many times the landmarks of local maps match, after loop closure.
   */
  struct LandmarkCorrespondenceManager {
		
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		
    typedef std::map<LandmarkCorrespondence, int> LandmarkCorrespondenceIntMap;

    LandmarkCorrespondenceManager(FeatureTracker* tracker_);
    int addCorrespondence(BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_, int k=1);
    bool removeCorrespondence(BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_);
    int occurrences (BaseTrackedLandmark* l1_, BaseTrackedLandmark* l2_);
    void mergeLandmarks(BaseTrackedLandmark* lkept, BaseTrackedLandmark* lremoved);
    inline size_t size() const {return _landmarkCorrespondenceMap.size();}

  /**This does the merging of the landmarks, updates the bookkeeping and does side effect in the tracker.
     @param minCount: the min number of times a landmark should have been matched
     @returns: the number of merged landmarks
   */
    int merge(int minCount);


  protected:
    FeatureTracker* _tracker;
    LandmarkCorrespondenceIntMap _landmarkCorrespondenceMap;
  };


  /**handles the optimization, wrapping the access from the tracker based structs to the 
     graph_based structs
   */
struct OptimizationManager{
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  OptimizationManager(FeatureTracker* tracker_, GraphItemSelector* graphItemSelector_);
  // initialize the optimization of a portion of the graph, keeping fixed gaugeFrame.
  // if push is true, the optimization is undone at cleanup
  void initializeLocal(BaseFrameSet& fset, BaseFrame* gaugeFrame = 0, bool push=true);
  // global optimization
  void initializeGlobal(BaseFrame* gaugeFrame = 0);  
  // do optimization
  void optimize(int iterations);  
  // always call this after initializeGlobal or initializeLocal
  void cleanup();
  
protected:
  FeatureTracker* _tracker;
  HyperGraph::EdgeSet _edges;
  OptimizableGraph::VertexSet _vertices;
  OptimizableGraph::Vertex* _gauge;
  bool _pushDone;
  GraphItemSelector* _graphItemSelector;
  void _initialize(BaseFrameSet& fset, bool push, BaseFrame* gaugeFrame = 0);
  bool _isInitialized;
};


 struct LandmarkDistanceEstimator{
	 
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   
   virtual bool compute(double& distance, BaseTrackedLandmark* l1, BaseTrackedLandmark* l2);
 };

/**
   Surprisingly handles the loop closures.
   <ul>
   <li>with the candidateDetector to look the frames where to try to close</li>
   <li>with the frameClusterer to determine which are the contiguous portion of the graph where to try to close</li>
   <li>for each cluster</li>

   <ul>
   <li> with the optimizer it optimizes the cluster (temporarily(</li>
   <li> with the correspondence finder it seeks for correspondences betweent he landmarks in the local map
   and the ones in the cluster</li>
   <li> with the matcher it determines the inliers and the outliers</li>
   <li> it updates the correspondence manager with the found matches<>
   </ul>
   </ul>
*/
 struct LoopClosureManager {
	 
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   
   LoopClosureManager(FeatureTracker* tracker_, 
		      LoopClosureCandidateDetector * closureCandidateDetector_,
		      FrameClusterer* frameClusterer_,
		      CorrespondenceFinder* correspondenceFinder_,
		      LandmarkCorrespondenceManager* correspondenceManager_,
		      Matcher* matcher_,
		      OptimizationManager* optimizationManager_,
		      GraphItemSelector* graphItemSelector_,
		      LandmarkDistanceEstimator* landmarkDistanceEstimator_);

   virtual void compute(BaseFrameSet& localFrames, BaseFrame* localMapGaugeFrame);

   inline int mergedLandmarks() const {return _mergedLandmarks;}
  
   inline int minFeaturesInCluster() const {return _minFeaturesInCluster;}
   inline void setMinFeaturesInCluster(int minFeaturesInCluster_) { _minFeaturesInCluster = minFeaturesInCluster_;}
   inline double closureInlierRatio() const {return _closureInlierRatio; }
   inline void setClosureInlierRatio(double closureInlierRatio_) {_closureInlierRatio = closureInlierRatio_;}
   inline int loopRansacIdentityMatches() const { return _loopRansacIdentityMatches; }
   inline void setLoopRansacIdentityMatches(int loopRansacIdentityMatches_) {_loopRansacIdentityMatches = loopRansacIdentityMatches_; }
   inline int localOptimizeIterations() const {return _localOptimizeIterations; }
   inline void setLocalOptimizeIterations(int localOptimizeIterations_) { _localOptimizeIterations = localOptimizeIterations_; }

   inline double landmarkMergeDistanceThreshold() const { return _landmarkMergeDistanceThreshold; }
   inline void setLandmarkMergeDistanceThreshold( double a) { _landmarkMergeDistanceThreshold = a; }

   inline const BaseFrameSet& touchedFrames() const {return _touchedFrames;}
   inline BaseFrameSet& touchedFrames() {return _touchedFrames;}

 protected:  
   FeatureTracker* _tracker; 
   LoopClosureCandidateDetector * _closureCandidateDetector;
   CorrespondenceFinder* _correspondenceFinder;
   LandmarkCorrespondenceManager* _correspondenceManager;
   Matcher* _matcher;
   OptimizationManager* _optimizationManager;
   GraphItemSelector* _graphItemSelector;
   FrameClusterer* _frameClusterer;
   LandmarkDistanceEstimator* _landmarkDistanceEstimator;
   // parameters
   double _landmarkMergeDistanceThreshold;
   int _minFeaturesInCluster;
   double _closureInlierRatio;
   int _loopRansacIdentityMatches;
   int _localOptimizeIterations;
   // state
   BaseFrameSet _touchedFrames;
   int _mergedLandmarks;
 };

}// end namespace

#endif
