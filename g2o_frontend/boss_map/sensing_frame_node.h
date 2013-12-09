#ifndef _BOSS_SENSING_FRAME_NODE_H_
#define _BOSS_SENSING_FRAME_NODE_H_

#include "map_core.h"
#include "robot_configuration.h"

namespace boss_map {
  using namespace boss;

  class MapManager;
  class MapNodeCollection;

  /***************************************** SensingFrame *****************************************/

  struct SensingFrameNode: public MapNode{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SensingFrameNode (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    //virtual void deserializeComplete();
    //! access to the sensor data
    inline std::vector<BaseSensorData*>& sensorDatas() {return _sensorDatas;}

    //! get (if it exists) a sensor by its topic
    BaseSensorData* sensorData(const std::string& topic_){
      for (size_t i=0; i<_sensorDatas.size(); i++)
	if(_sensorDatas[i]->topic()==topic_)
	  return _sensorDatas[i];
      return 0;
    }

    //! access to the frame shared by the sensor data
    ReferenceFrame* robotFrame();
  protected:
    std::vector<BaseSensorData*> _sensorDatas;
  };

  class SensingFrameNodeMaker {
  public:
    SensingFrameNodeMaker();
    void init(MapManager* manager_, RobotConfiguration* config_);
    SensingFrameNode* processData(BaseSensorData* data);

  protected:
    MapManager* _mapManager;
    RobotConfiguration* _config;
    SensingFrameNode* _currentSensingFrameNode;
    BaseSensorData* _previousData;
  };

}

#endif
