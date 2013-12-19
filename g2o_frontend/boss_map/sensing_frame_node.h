#ifndef _BOSS_SENSING_FRAME_NODE_H_
#define _BOSS_SENSING_FRAME_NODE_H_

#include "map_core.h"
#include "robot_configuration.h"
#include "stream_processor.h"
#include "imu_sensor.h"

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

    // !previous node getter
    inline SensingFrameNode* previousNode() const {return _previousNode;}

    // !previous node setter
    inline void setPreviousNode(SensingFrameNode* previousNode_)  {_previousNode = previousNode_;}
    
    //! odometry getter
    inline MapNodeBinaryRelation* odometry() const { return _odometry;}
    
    //! odometry setter
    inline void setOdometry(MapNodeBinaryRelation* odometry_)  { _odometry = odometry_;}

    //! imu getter
    inline MapNodeUnaryRelation* imu() const { return _imu;}
    
    //! imu setter
    inline void setImu(MapNodeUnaryRelation* imu_) { _imu = imu_;}

  protected:
    std::vector<BaseSensorData*> _sensorDatas;
    MapNodeBinaryRelation* _odometry;
    MapNodeUnaryRelation* _imu;
    SensingFrameNode* _previousNode;
  };

  class SensingFrameNodeMaker: public StreamProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SensingFrameNodeMaker();
    void init(MapManager* manager_, RobotConfiguration* config_);
    virtual void process(Serializable* s);
  protected:
    SensingFrameNode* processData(BaseSensorData* data);
    MapManager* _mapManager;
    RobotConfiguration* _config;
    BaseSensorData* _previousData;
    SensingFrameNode* _currentNode, *_previousNode;
    Eigen::Isometry3d _currentNodeTransform, _previousNodeTransform;
    MapNodeUnaryRelation* _lastImu;
    MapNodeBinaryRelation* _lastOdom;
    int _seq;
    std::list<Serializable*> _outputQueue;
    void flushQueue();
  };

}

#endif
