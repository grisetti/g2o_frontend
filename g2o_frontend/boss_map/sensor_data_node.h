#pragma once

#include "map_core.h"
#include "robot_configuration.h"
#include "stream_processor.h"
#include "imu_sensor.h"
#include "sensor_data_synchronizer.h"

namespace boss_map {
  using namespace boss;

  class MapManager;
  class MapNodeCollection;

  /***************************************** SensingFrame *****************************************/

  struct BaseSensorDataNode: public MapNode{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    BaseSensorDataNode (MapManager* manager=0, BaseSensorData* data = 0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);

    // !previous node getter
    inline BaseSensorDataNode* previousNode() const {return _previousNode;}

    // !previous node setter
    inline void setPreviousNode(BaseSensorDataNode* previousNode_)  {_previousNode = previousNode_;}
    
    //! odometry getter
    inline MapNodeBinaryRelation* odometry() const { return _odometry;}
    
    //! odometry setter
    inline void setOdometry(MapNodeBinaryRelation* odometry_)  { _odometry = odometry_;}


  protected:
    BaseSensorData* _sensorData;
    MapNodeBinaryRelation* _odometry;
    BaseSensorDataNode* _previousNode;
  };

  template <typename SensorDataType_>
  class SensorDataNode: public BaseSensorDataNode {
  public:
    typedef SensorDataType_ SensorDataType;
    SensorDataNode(MapManager* manager=0, SensorDataType* data=0, int id=-1, IdContext* context = 0):
      BaseSensorDataNode(manager, data, id, context) {}

    inline SensorDataType* sensorData() { 
      return 
	_sensorData ?
	dynamic_cast<SensorDataType*>(_sensorData):
	0;
    }

    inline void setSensorData(SensorDataType* s) {  _sensorData = s;}
  };
  

  class SensorDataNodeMaker: public StreamProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SensorDataNodeMaker(MapManager* manager_, RobotConfiguration* config_);
    virtual void process(Serializable* s);
    virtual BaseSensorDataNode* makeNode(MapManager* manager, BaseSensorData* data);
    const std::string topic() { return _topic; }
    void setTopic(const std::string& topic_) { _topic = topic_; }
  protected:
    std::string _topic;
    MapManager* _mapManager;
    RobotConfiguration* _config;
    BaseSensorDataNode* _previousNode;
    Eigen::Isometry3d _previousNodeTransform;
    int _seq;
  };


  class SyncSensorDataNode: public SensorDataNode<SynchronizedSensorData> {
  public:
    SyncSensorDataNode(MapManager* manager=0, SynchronizedSensorData* data=0, int id=-1, IdContext* context = 0):
      SensorDataNode<SynchronizedSensorData>(manager, data, id, context) {}
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);

    //! imu getter
    inline MapNodeUnaryRelation* imu() const { return _imu;}
    
    //! imu setter
    inline void setImu(MapNodeUnaryRelation* imu_) { _imu = imu_;}
  protected:
    MapNodeUnaryRelation* _imu;
  };

  class SyncSensorDataNodeMaker: public SensorDataNodeMaker {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SyncSensorDataNodeMaker(MapManager* manager_, RobotConfiguration* config_);
    virtual void process(Serializable* s);
    virtual BaseSensorDataNode* makeNode(MapManager* manager, BaseSensorData* data);
  };

}
