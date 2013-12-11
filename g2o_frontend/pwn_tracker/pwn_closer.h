#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/pwn_core/frame.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/pwn_core/aligner.h"
#include "g2o_frontend/boss_map_building/boss_map_g2o_reflector.h"
#include "g2o_frontend/boss_map_building/map_closer.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include "pwn_tracker.h"
#include "map_g2o_wrapper.h"

namespace pwn_tracker {
  using namespace std;
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;
  using namespace boss_map_building;
  using namespace pwn_tracker;
  
  struct PwnCloserRelation: public PwnTrackerRelation, ClosureInfo {
    PwnCloserRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);


    // matching result parameters
    float normalDifference;
    float depthDifference;
    float reprojectionDistance;
    int nonZeros;
    int outliers;
    int inliers;
    cv::Mat  diffRegistered;
  };

  class PwnCloser: public boss_map_building::MapCloser{
  public:

    PwnCloser(pwn::Aligner* aligner_, 
	      pwn::DepthImageConverter* converter_,
	      MapManager* manager_,
	      PwnCache* cache_);
    
    inline pwn::Aligner* aligner() { return _aligner;}
    inline void setAligner(pwn::Aligner* aligner_) { _aligner=aligner_;}

    inline pwn::DepthImageConverter* converter() { return _converter;}
    inline void setConverter(pwn::DepthImageConverter* converter_) { _converter=converter_; updateCache();}

    inline boss_map::PoseAcceptanceCriterion* criterion() {return _criterion;}
    void setCriterion(boss_map::PoseAcceptanceCriterion* criterion_) { _criterion= criterion_;}

    inline int scale() const {return _scale;}
    inline void setScale(int scale_) {_scale = scale_; updateCache();}
    inline PwnCache* cache() {return _cache;}

    virtual void processPartition(std::set<MapNode*> & otherPartition, MapNode* current_);
    PwnCloserRelation* matchFrames(PwnTrackerFrame* from, PwnTrackerFrame* to, 
				    pwn::Frame* fromCloud, pwn::Frame* toCloud,
				    const Eigen::Isometry3d& initialGuess);
  protected:
    void updateCache();
    static float compareNormals(cv::Mat& m1, cv::Mat& m2);
    static float compareDepths(cv::Mat& m1, cv::Mat& m2);

    int _scale;
    pwn::DepthImageConverter* _converter;
    pwn::Aligner* _aligner;
    PwnCache* _cache;
    float _frameInlierDepthThreshold;
    int _frameMinNonZeroThreshold;
    int _frameMaxOutliersThreshold;
    int _frameMinInliersThreshold;
  private:
    void scoreMatch(PwnCloserRelation* rel);
  };

  class PwnCloserActiveRelationSelector: public MapRelationSelector {
  public:
    PwnCloserActiveRelationSelector(boss_map::MapManager* manager);
    virtual bool accept(MapNodeRelation* r);
  };

// closure actions

struct NewFrameCloserAdder: public PwnTracker::NewFrameAction {
  NewFrameCloserAdder(PwnCloser* closer, PwnTracker* tracker);
  void compute (PwnTrackerFrame* frame);
  PwnCloser* _closer;
};


  struct CloserRelationAdder: public PwnTracker::NewRelationAction {
    CloserRelationAdder(std::list<Serializable*>& objects_,
			PwnCloser* closer, 
			G2oWrapper* optimizer_, 
			PwnTracker* tracker);
    void compute (PwnTrackerRelation* relation);
  protected:
    PwnCloser* _closer;
    G2oWrapper* _optimizer;
    std::list<Serializable*>& _objects;
  };
}
