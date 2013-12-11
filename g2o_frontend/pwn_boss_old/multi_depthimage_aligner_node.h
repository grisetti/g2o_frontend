#ifndef _TWO_DEPTH_IMAGE_ALIGNER_NODE_H_
#define _TWO_DEPTH_IMAGE_ALIGNER_NODE_H_

#include "g2o_frontend/pwn_core/frame.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/pwn_core/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/boss_map/reference_frame_relation.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include "g2o_frontend/boss_map/laser_sensor.h"
#include "g2o_frontend/boss_map/imu_sensor.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "g2o_frontend/boss_map/map_core.h"
#include "g2o_frontend/boss_map/sensing_frame_node.h"
#include "g2o_frontend/boss_map/map_node_processor.h"


namespace pwn_boss {
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;

  class MultiDepthImageAlignerNode : public TwoDepthImageAlignerNode {
  public:
    
    MultiDepthImageAlignerNode(MapManager* manager_,
			     RobotConfiguration* config_,
			     DepthImageConverter* converter_,
			     Aligner* aligner_,
			     const std::string& topic_);

    virtual void processNode(MapNode* node_);

  protected:
    DepthImageConverter* _converter;
    Aligner* _aligner;
    Merger* _merger;
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
