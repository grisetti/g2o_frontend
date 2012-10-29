#ifndef G2O_FRONTEND_FEATURE_TRACKER_POINTXY
#define G2O_FRONTEND_FEATURE_TRACKER_POINTXY
#include <set>
#include "feature_tracker.h"
#include "feature_tracker_closure.h"

#include "g2o_frontend/data/feature_data.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam2d/types_slam2d.h"

namespace g2o {
 
  struct PointXYLandmarkConstructor: public LandmarkConstructor{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
      virtual BaseTrackedLandmark* constructLandmark(std::vector<BaseTrackedFeature*> &);
    virtual OptimizableGraph::Edge* constructEdge(BaseTrackedLandmark* l, BaseTrackedFeature* f);
    virtual int minNumObservations() const {return 1;}
  };

  enum FeatureMappingMode{UseRobotPose,UseLandmark,UseLandmarkIfAvailable,LocalFrame};
  bool remapPose(Vector2d& newPose, Matchable* trackedFeature, FeatureMappingMode mode);
  
  typedef std::map<BaseTrackedFeature*, Eigen::Vector2d, 
    std::less<BaseTrackedFeature*>,
    Eigen::aligned_allocator<std::pair<BaseTrackedFeature*, Eigen::Vector2d> > > FeaturePoseMap;
  
  
  struct PointXYInitialGuessCorrespondenceFinder: public CorrespondenceFinder {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
      PointXYInitialGuessCorrespondenceFinder();
    
    // determines candidate correspondences between two sets of features;
    virtual void compute(BaseTrackedFeatureSet& s1, BaseTrackedFeatureSet& s2);

    inline void setDistanceThreshold(double dt) { _squaredDistanceThreshold = dt*dt; }

    double distanceThreshold() const { return sqrt(_squaredDistanceThreshold); }

    inline FeatureMappingMode featureMappingMode(int i) {
      return _featureMappingMode[i];
    }

    inline void setFeatureMappingMode(int i, FeatureMappingMode mode){
      _featureMappingMode[i]=mode;
    }
 
  protected:
    double _squaredDistanceThreshold;
    FeatureMappingMode _featureMappingMode[2];
  };

  struct PointXYRansacMatcher: public Matcher{
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
      PointXYRansacMatcher();
    virtual void compute(const CorrespondenceVector& correspondences);  
    inline const CorrespondenceVector& matches() {return _matches;}
    //virtual bool applyCandidateTransform();
  
    inline void setMaxIterationsThreshold(double a) { _maxIterationsThreshold = a; }
    inline double maxIterationsThreshold() const { return _maxIterationsThreshold; }

    inline void setMinIterationsThreshold(double a) { _minIterationsThreshold = a; }
    inline double minIterationsThreshold() const { return _minIterationsThreshold; }

    inline void setInliersThreshold(double a) { _inliersThreshold = a; }
    inline double inliersThreshold() const { return _inliersThreshold; }

    inline void setIntraFrameDistanceDifferenceThreshold(double a) { _squaredIntraFrameDistanceDifferenceThreshold = a*a; }
    inline double intraFrameDistanceDifferenceThreshold() const { return _squaredIntraFrameDistanceDifferenceThreshold; }

    inline void setInterFrameDistanceDifferenceThreshold(double a) { _squaredInterFrameDistanceDifferenceThreshold = a*a; }
    inline double interFrameDistanceDifferenceThreshold() const { return _squaredInterFrameDistanceDifferenceThreshold; }

    inline void setInlierDistanceThreshold(double a) { _squaredInlierDistanceThreshold = a*a; }
    double inlierDistanceThreshold() const { return _squaredInlierDistanceThreshold; }

    inline FeatureMappingMode featureMappingMode(int i) {
      return _featureMappingMode[i];
    }

    inline void setFeatureMappingMode(int i, FeatureMappingMode mode){
      _featureMappingMode[i]=mode;
    }

    inline SE2 transformation() {
      return _transformation;
    }

    inline int numInliers() {
      return _numInliers;
    }

    inline double error() {
      return _error;
    }


  protected:
    double _maxIterationsThreshold;
    double _minIterationsThreshold;
    double _inliersThreshold;
    double _squaredIntraFrameDistanceDifferenceThreshold;
    double _squaredInterFrameDistanceDifferenceThreshold;
    double _squaredInlierDistanceThreshold;
    FeatureMappingMode _featureMappingMode[2];
    
    SE2 _transformation;
    int _numInliers;
    double _error;
  };

  struct SE2LoopClosureCandidateDetector: public LoopClosureCandidateDetector {
	  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
      SE2LoopClosureCandidateDetector(MapperState* mapperState_);
    virtual void compute(BaseFrame* frame);
    inline void setLinearClosureThreshold(double t) { _squaredLinearClosureThreshold = t*t; }
    inline void setAngularClosureThreshold(double t) { _angularClosureThreshold = t*t; }
    double linearClosureThreshold() const {return sqrt(_squaredLinearClosureThreshold); }
    double angularClosureThreshold() const {return _angularClosureThreshold; }

  protected:
    double _squaredLinearClosureThreshold;
    double _angularClosureThreshold;

  };

  struct PointXYLandmarkDistanceEstimator : public LandmarkDistanceEstimator {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      virtual bool compute(double& distance, BaseTrackedLandmark* l1, BaseTrackedLandmark* l2);
  };

}// end namespace

#endif
