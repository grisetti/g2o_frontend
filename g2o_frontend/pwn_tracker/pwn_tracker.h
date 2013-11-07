#ifndef _PWN_TRACKER_H_
#define _PWN_TRACKER_H_

#include "g2o_frontend/boss_map/boss_map.h"
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "opencv2/core/core.hpp"
#include <fstream>
#include <iostream>

namespace pwn_tracker{

using namespace std;
using namespace boss;
using namespace boss_logger;
using namespace boss_map;
using namespace pwn;


  struct PwnTrackerFrame: public boss_map::MapNode {
    PwnTrackerFrame (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);


    pwn::FrameBLOBReference cloud;
    boss_logger::ImageBLOBReference depthImage;
    int imageRows, imageCols;
    boss_logger::ImageBLOBReference normalThumbnail;
    boss_logger::ImageBLOBReference depthThumbnail;
    Eigen::Isometry3f sensorOffset;
    Eigen::Matrix3f cameraMatrix;
    float scale;
  };


  struct PwnTrackerRelation: public MapNodeBinaryRelation{
    PwnTrackerRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    inline PwnTrackerFrame* from() { return static_cast<PwnTrackerFrame*>(_nodes[0]); }
    inline PwnTrackerFrame* to()   { return static_cast<PwnTrackerFrame*>(_nodes[1]); }
    void setFrom(PwnTrackerFrame* from_) {_nodes[0] = from_; }
    void setTo(PwnTrackerFrame* to_) {_nodes[1] = to_; }
    int inliers;
    float error;
  };


  struct PwnTracker{
    PwnTracker(pwn::Aligner* aligner, pwn::DepthImageConverter* converter, boss_map::MapManager* manager);

    void makeThumbnails(cv::Mat& depthThumbnail, cv::Mat& normalThumbnail, 
			Frame* f, int r, int c, 
			const Eigen::Isometry3f& offset, 
			const Eigen::Matrix3f& cameraMatrix,
			float scale);

    float compareDepthThumbnails();
    float compareNormalThumbnails();
    pwn::Frame* makeCloud(int& r, int& c,
			  Eigen::Matrix3f& cameraMatrix, 
			  const Eigen::Isometry3f& sensorOffset, 
			  const DepthImage& depthImage);

    virtual void processFrame(const pwn::DepthImage& depthImage, 
			      const Eigen::Isometry3f& sensorOffset, 
			      const Eigen::Matrix3f& cameraMatrix,
			      const Eigen::Isometry3f& initialGuess=Eigen::Isometry3f::Identity());

    virtual ~PwnTracker();  
  
    virtual void newFrameCallback(PwnTrackerFrame* frame) {}
    virtual void newAlignmentCallback(const Eigen::Isometry3f& globalT, 
				      const Eigen::Isometry3f& localT, 
				      int inliers, float error ) {}
    virtual void newRelationCallback(PwnTrackerRelation* relation) {}
    virtual void initCallback() {}
    
    void init();

    Aligner* _aligner;
    DepthImageConverter* _converter;
    int _scale;
    pwn::Frame* _previousCloud;
    PwnTrackerFrame* _previousTrackerFrame;
    Eigen::Isometry3f _previousCloudTransform;
    Eigen::Isometry3f _globalT;
    int _counter;
    int _numKeyframes;
    float _newFrameInliersFraction;
    MapManager* _manager;
  };


  template <typename T1, typename T2>
  void convertScalar(T1& dest, const T2& src){
    for (int i=0; i<src.matrix().cols(); i++)
      for (int j=0; j<src.matrix().rows(); j++)
	dest.matrix()(j,i) = src.matrix()(j,i);

  }

  std::vector<Serializable*> readConfig(Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile);

}// end namespace


#endif
