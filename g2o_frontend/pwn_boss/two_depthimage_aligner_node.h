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


  class TwoDepthImageAlignerNodeRelation : public MapNodeBinaryRelation{
    TwoDepthImageAlignerNodeRelation(MapManager* manager, int id = -1, IdContext* = 0);
    Aligner* _aligner;
    DepthImageConverter* _converter;
    ImageData* image1;
    ImageData* image2;
    int inliners;
    float error;
  };


  class TwoDepthImageAlignerNode : public MapNodeProcessor, public Identifiable {
  public:
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
