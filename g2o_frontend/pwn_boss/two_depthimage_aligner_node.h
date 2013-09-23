#ifndef _TWO_DEPTH_IMAGE_ALIGNER_NODE_H_
#define _TWO_DEPTH_IMAGE_ALIGNER_NODE_H_

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/boss_logger/bframerelation.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "g2o_frontend/boss_logger/blasersensor.h"
#include "g2o_frontend/boss_logger/bimusensor.h"
#include "g2o_frontend/boss_logger/brobot_configuration.h"

#include "g2o_frontend/boss_map/boss_map.h"
#include "g2o_frontend/boss_map/sensing_frame_node.h"
#include "g2o_frontend/boss_map/map_node_processor.h"


namespace pwn_boss {
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;
  using namespace boss_logger;




  class TwoDepthImageAlignerNode : public MapNodeProcessor, public Identifiable {
  public:
    
    class Relation : public MapNodeBinaryRelation{
    public:
      Relation(Aligner* aligner_=0,
	       DepthImageConverter* converter_=0,
	       SensingFrameNode* referenceSensingFrame_=0,
	       SensingFrameNode* currentSensingFrame_=0,
	       const std::string& topic_ ="",
	       int inliers_ = 0,
	       int error_ = 0,
	       MapManager* manager = 0, 
	       int id = -1, 
	       IdContext* context= 0);
      inline Aligner* aligner() {return _aligner;}
      inline DepthImageConverter* converter() {return _converter;}
      SensingFrameNode* referenceSensingFrame() {return _referenceSensingFrame;}
      SensingFrameNode* currentSensingFrame() {return _currentSensingFrame;}
      const std::string& topic() { return _topic;}
      int inliers() const {return _inliers;}
      float error() const { return _error;}

      virtual void serialize(ObjectData& data, IdContext& context);
      virtual void deserialize(ObjectData& data, IdContext& context);

    protected:
      Aligner* _aligner;
      DepthImageConverter* _converter;
      SensingFrameNode* _referenceSensingFrame;
      SensingFrameNode* _currentSensingFrame;
      std::string _topic;
      int _inliers;
      float _error;
    };

    TwoDepthImageAlignerNode(MapManager* manager_,
			     RobotConfiguration* config_,
			     DepthImageConverter* converter_,
			     Aligner* aligner_,
			     const std::string& topic_);

    virtual void processNode(MapNode* node_);

  protected:
    DepthImageConverter* _converter;
    Aligner* _aligner;
    SensingFrameNode* _previousSensingFrameNode;
    pwn::Frame* _previousFrame;
    std::string _topic;
    BaseSensor* _sensor;
    int _scale;
    Eigen::Isometry3f _globalT;
    ofstream* os;
    int _counter;
  };

}

#endif
