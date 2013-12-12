#pragma once
#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/pwn_core/frame.h"
#include "g2o_frontend/boss_map/image_sensor.h"

namespace pwn_tracker{

using namespace std;
using namespace boss;
using namespace boss_map;
using namespace pwn;


  struct PwnTrackerFrame: public boss_map::MapNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PwnTrackerFrame (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    pwn::Frame* cloud;
    boss_map::ImageBLOBReference depthImage;
    int imageRows, imageCols;
    boss_map::ImageBLOBReference normalThumbnail;
    boss_map::ImageBLOBReference depthThumbnail;
    Eigen::Isometry3f sensorOffset;
    Eigen::Matrix3f cameraMatrix;
    float scale;
  };
}
