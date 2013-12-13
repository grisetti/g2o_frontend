#pragma once
#include "pwn_tracker_frame.h"
#include "pwn_tracker_relation.h"
#include "pwn_tracker_cache.h"
#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "pwn_matcher_base.h"
#include "opencv2/core/core.hpp"
#include <fstream>
#include <iostream>
#include <list>

namespace pwn_tracker{

using namespace std;
using namespace boss;
using namespace boss_map;
using namespace pwn;

  struct PwnTracker:  public PwnMatcherBase{

    struct PwnTrackerAction{
      PwnTrackerAction(PwnTracker* tracker_);
      virtual ~PwnTrackerAction() {};
    protected:
      PwnTracker* _tracker;
    };
    struct NewFrameAction: public PwnTrackerAction{
      NewFrameAction(PwnTracker* tracker_);
      virtual void compute(PwnTrackerFrame* frame) = 0;
    };
    struct AlignmentAction: public PwnTrackerAction{
      AlignmentAction(PwnTracker* tracker_);
      virtual void compute(const Eigen::Isometry3f& globalT, 
			   const Eigen::Isometry3f& localT, 
			   int inliers, float error ) = 0;
    };
    struct NewRelationAction: public PwnTrackerAction{
      NewRelationAction(PwnTracker* tracker_);
      virtual void compute(PwnTrackerRelation* rel)=0;
    };
    struct InitAction: public PwnTrackerAction{
      InitAction(PwnTracker* tracker_);
      virtual void compute()=0;
    };

    PwnTracker(pwn::Aligner* aligner, pwn::DepthImageConverter* converter, boss_map::MapManager* manager, PwnCache* cache=0);


    virtual void processFrame(const pwn::DepthImage& depthImage, 
			      const Eigen::Isometry3f& sensorOffset, 
			      const Eigen::Matrix3f& cameraMatrix,
			      const Eigen::Isometry3f& initialGuess=Eigen::Isometry3f::Identity());

    virtual ~PwnTracker();  
    void init();
    inline float newFrameInliersFraction() const {return _newFrameInliersFraction;}
    inline void setNewFrameInliersFraction(float nf) {_newFrameInliersFraction = nf;}
    std::list<InitAction*>& initActions() {return _initActions;}
    std::list<AlignmentAction*>& alignmentActions() {return _alignmentActions;}
    std::list<NewRelationAction*>& newRelationActions() { return _newRelationActions;}
    std::list<NewFrameAction*>& newFrameActions() {return _newFrameActions;}

    PwnCache* cache() const {return _cache;}
  protected:  
    //PwnTrackerFrame* makeFrame();
    virtual void newFrameCallbacks(PwnTrackerFrame* frame);
    virtual void newAlignmentCallbacks(const Eigen::Isometry3f& globalT, 
				      const Eigen::Isometry3f& localT, 
				       int inliers, float error );
    virtual void newRelationCallbacks(PwnTrackerRelation* relation); 
    virtual void initCallbacks();

    std::list<InitAction*> _initActions;
    std::list<AlignmentAction*> _alignmentActions;
    std::list<NewRelationAction*> _newRelationActions;
    std::list<NewFrameAction*> _newFrameActions;

    pwn::Frame* _previousCloud;
    PwnCache::HandleType _previousCloudHandle;
    PwnTrackerFrame* _previousTrackerFrame;
    Eigen::Isometry3f _previousCloudTransform;
    Eigen::Isometry3f _previousCloudOffset;

    pwn::Frame* _currentCloud;
    PwnCache::HandleType _currentCloudHandle;
    PwnTrackerFrame* _currentTrackerFrame;
    Eigen::Matrix3f  _currentCameraMatrix;
    Eigen::Isometry3f _currentCloudTransform;
    Eigen::Isometry3f _currentCloudOffset;
    pwn::DepthImage _currentDepthImage;

    Eigen::Isometry3f _globalT;
    int _counter;
    int _numKeyframes;
    float _newFrameInliersFraction;
    MapManager* _manager;
    PwnCache* _cache;
    int _seq;
  };

}// end namespace

