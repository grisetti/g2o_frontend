#ifndef _PWN_CLOSER_H_
#define _PWN_CLOSER_H_

#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss_map/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss_map_building/boss_map_g2o_reflector.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "pwn_tracker.h"
#include "cache.h"


namespace pwn_tracker {
  using namespace std;
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;
  using namespace boss_map_building;
  using namespace pwn_tracker;


  struct PwnCloserRelation: public PwnTrackerRelation {
    PwnCloserRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);

    // status
    bool accepted;
    int consensusCumInlier;
    int consensusCumOutlierTimes;
    int consensusTimeChecked;

    // matching result parameters
    float normalDifference;
    float depthDifference;
    float reprojectionDistance;
    int nonZeros;
    int outliers;
    int inliers;
    cv::Mat  diffRegistered;
  };

  class PwnCloser{
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
    void addFrame(PwnTrackerFrame* f);
    void addRelation(PwnTrackerRelation* r);
    void process();
    void processPartition(std::set<MapNode*> & otherPartition, MapNode* current_);
    PwnCloserRelation* matchFrames(PwnTrackerFrame* from, PwnTrackerFrame* to, 
				    pwn::Frame* fromCloud, pwn::Frame* toCloud,
				    const Eigen::Isometry3d& initialGuess);
    std::list<PwnCloserRelation*>& results() {return _results;}
    std::vector<PwnTrackerRelation*> _trackerRelations;
    std::map<int, PwnTrackerFrame*> _trackerFrames;

    std::list<PwnCloserRelation*>& committedRelations() {return _committedRelations;}
    std::list<PwnCloserRelation*>& candidateRelations() {return _candidateRelations;}
    std::vector<std::set<MapNode*> >& partitions() {return _partitions;}
    std::set<MapNode*>* currentPartition() {return _currentPartition;}
    bool _debug;
  protected:
    void updateCache();
    static float compareNormals(cv::Mat& m1, cv::Mat& m2);
    static float compareDepths(cv::Mat& m1, cv::Mat& m2);
    void validatePartitions(std::set<MapNode*>& other, 
			    std::set<MapNode*>& current);

    std::vector<std::set<MapNode*> > _partitions;
    std::set<MapNode*>* _currentPartition;

    float _consensusInlierTranslationalThreshold;
    float _consensusInlierRotationalThreshold;
    int    _consensusMinTimesCheckedThreshold;

    int _scale;
    PwnTrackerFrame* _pendingTrackerFrame, *_lastTrackerFrame;
    boss_map::MapManager* _manager;
    pwn::DepthImageConverter* _converter;
    pwn::Aligner* _aligner;
    PwnCache* _cache;
    PoseAcceptanceCriterion* _criterion;
    std::list<PwnCloserRelation*> _results;
    float _frameInlierDepthThreshold;
    int _frameMinNonZeroThreshold;
    int _frameMaxOutliersThreshold;
    int _frameMinInliersThreshold;
    std::list<PwnCloserRelation*> _committedRelations;
    std::list<PwnCloserRelation*> _candidateRelations;
  private:
    void scoreMatch(PwnCloserRelation* rel);
  };
}

#endif
