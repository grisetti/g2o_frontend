#ifndef _PWN_CLOSER_H_
#define _PWN_CLOSER_H_

#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss_map/boss_map_g2o_reflector.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "pwn_tracker.h"
#include "cache.h"


namespace pwn_tracker {
  using namespace std;
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;
  using namespace pwn_tracker;

  struct PwnCloserRelation: public PwnTrackerRelation {
    PwnCloserRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    bool accepted;
    int cumInlier;
    int cumOutlierTimes;
    int timesChecked;
    int reprojectionInliers;
    int reprojectionOutliers;
  };

  struct MatchingResult{
    MatchingResult();
    PwnTrackerFrame* from;
    PwnTrackerFrame* to;
    PwnCloserRelation* relation;
    float normalDifference;
    float depthDifference;
    float reprojectionDistance;
    float nonZeros;
    int outliers;
    int inliers;
    cv::Mat  diffRegistered;
    cv::Mat  normalsRegistered;
  };

  // struct RelationValidator{
  //   std::set<PwnTrackerFrame*> referenceSet;
  //   std::set<PwnTrackerFrame*> currentSet;
  //   std::set<PwnTrackerRelation*> relations;
  //   PwnTrackerFrame* currentFrame;
  //   std::vector<Eigen::Isometry3d, Eigen::allocator<Isometry3d> > transforms;
  //   void saveTransforms();
  //   void restoreTransforms();
  //   void validateRelation(PwnTrackerFrameRelation*);
  // }

  class PwnCloser{
  public:

    class AcceptanceCriterion{
    public:
      AcceptanceCriterion(PwnCloser* closer_) {_closer = closer_;}
      inline PwnCloser* closer() {return  _closer;}
      virtual bool accept(const MatchingResult& result);
    protected:
      PwnCloser* _closer;
    };

    PwnCloser(pwn::Aligner* aligner_, 
	      pwn::DepthImageConverter* converter_,
	      MapManager* manager_);
    
    inline pwn::Aligner* aligner() { return _aligner;}
    inline void setAligner(pwn::Aligner* aligner_) { _aligner=aligner_;}

    inline pwn::DepthImageConverter* converter() { return _converter;}
    inline void setConverter(pwn::DepthImageConverter* converter_) { _converter=converter_; updateCache();}

    inline boss_map::PoseAcceptanceCriterion* criterion() {return _criterion;}
    void setCriterion(boss_map::PoseAcceptanceCriterion* criterion_) { _criterion= criterion_;}

    inline int scale() const {return _scale;}
    inline void setScale(int scale_) {_scale = scale_; updateCache();}
    inline PwnCache* cache() {return _cache;}
    void addFrame(PwnTrackerFrame* f);
    void addRelation(PwnTrackerRelation* r);
    void process();
    void processPartition(std::set<MapNode*> & otherPartition, MapNode* current_);
    bool matchFrames(MatchingResult& result,
		     PwnTrackerFrame* from, PwnTrackerFrame* to, 
		     pwn::Frame* fromCloud, pwn::Frame* toCloud,
		     const Eigen::Isometry3d& initialGuess);
    std::vector<MatchingResult>& results() {return _results;}
    void scoreCandidate(MatchingResult& candidate);
    std::vector<PwnTrackerRelation*> _trackerRelations;
    std::map<int, PwnTrackerFrame*> _trackerFrames;

    int committedRelations() const {return _committedRelations;}
  protected:
    void updateCache();
    static float compareNormals(cv::Mat& m1, cv::Mat& m2);
    static float compareDepths(cv::Mat& m1, cv::Mat& m2);
    void validatePartitions(PwnTrackerFrame* currentNode,
			    int partitionIndex,
			    std::set<MapNode*>& other, 
			    std::set<MapNode*>& current);
  
    int _scale;
    PwnTrackerFrame* _pendingTrackerFrame, *_lastTrackerFrame;
    boss_map::MapManager* _manager;
    pwn::DepthImageConverter* _converter;
    pwn::Aligner* _aligner;
    PwnCache* _cache;
    PoseAcceptanceCriterion* _criterion;
    std::vector<MatchingResult> _results;
    float _frameInlierDepthThreshold;
    int _frameMinNonZeroThreshold;
    int _frameMaxOutliersThreshold;
    int _frameMinInliersThreshold;
    int _committedRelations;
  };
}

#endif
