#include "sensor_data_node.h"
#include "map_manager.h"
#include <stdexcept>
#include <iostream>
#include "g2o_frontend/basemath/bm_se3.h"

namespace boss_map {
  using namespace std;
  using namespace boss;
  /***************************************** SensingFrame *****************************************/

  BaseSensorDataNode::BaseSensorDataNode (MapManager* manager, BaseSensorData* data, int id, IdContext* context):
    MapNode(manager, id, context){
    _odometry = 0;
    //_imu = 0;
    _previousNode = 0;
    _sensorData = data;
  }
  
  void BaseSensorDataNode::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    //data.setPointer("imu",_imu);
    data.setPointer("odometry", _odometry);
    data.setPointer("previousNode", _previousNode);
    data.setPointer("sensorData", _sensorData);
  }

  void BaseSensorDataNode::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
    //data.getReference("imu").bind(_imu);
    data.getReference("odometry").bind(_odometry);
    data.getReference("previousNode").bind(_previousNode);
    data.getReference("sensorData").bind(_sensorData);
  }



  SensorDataNodeMaker::SensorDataNodeMaker(MapManager* manager_, RobotConfiguration* config_){
    _mapManager = manager_;
    _config = config_;
    _previousNode = 0;
    _seq = 0;
    _topic = "";
  }

  BaseSensorDataNode* SensorDataNodeMaker::makeNode(MapManager* manager, BaseSensorData* data){
    return new BaseSensorDataNode(manager, data);
  }


  void SensorDataNodeMaker::process(Serializable* s){
    put(s);
    BaseSensorData* data = dynamic_cast<BaseSensorData*>(s);
    if (data && data->topic() == _topic) {
      BaseSensorDataNode* currentNode = makeNode(_mapManager, data);
      Eigen::Isometry3d currentNodeTransform = data->robotReferenceFrame()->transform();
      currentNode->setTransform(currentNodeTransform);
      _mapManager->addNode(currentNode);
      MapNodeBinaryRelation* odom = 0;
      if (_previousNode) {
	  odom = new MapNodeBinaryRelation(_mapManager);
	  odom->nodes()[0]=_previousNode;
	  odom->nodes()[1]=currentNode;
	  odom->setTransform(_previousNodeTransform.inverse()*currentNodeTransform);	
	  Eigen::Matrix<double, 6, 6> info;
	  info.setIdentity();
	  info = info * 100;
	  odom->setInformationMatrix(info);
	  _mapManager->addRelation(odom);
	  currentNode->setOdometry(odom);
	  currentNode->setPreviousNode(_previousNode);
      }
      put(currentNode);
      if (odom)
	put(odom);
      _previousNode = currentNode;
      _previousNodeTransform = currentNodeTransform;
    }
  }


  void SyncSensorDataNode::serialize(ObjectData& data, IdContext& context) {
    SensorDataNode<SynchronizedSensorData>::serialize(data, context);
    data.setPointer("imu",_imu);
  }
    
  void SyncSensorDataNode::deserialize(ObjectData& data, IdContext& context){
    SensorDataNode<SynchronizedSensorData>::deserialize(data, context);
    data.getReference("imu").bind(_imu);
  }

  
  SyncSensorDataNodeMaker::SyncSensorDataNodeMaker(MapManager* manager_, RobotConfiguration* config_):
    SensorDataNodeMaker(manager_, config_){
  }

  void SyncSensorDataNodeMaker::process(Serializable* s){
    put(s);
    BaseSensorData* data = dynamic_cast<BaseSensorData*>(s);
    if (data && data->topic() == _topic) {
      SyncSensorDataNode* currentNode = (SyncSensorDataNode*) makeNode(_mapManager, data);
      currentNode->setSeq(_seq);
      _seq++;
      Eigen::Isometry3d currentNodeTransform = data->robotReferenceFrame()->transform();
      currentNode->setTransform(currentNodeTransform);
      _mapManager->addNode(currentNode);
      MapNodeBinaryRelation* odom = 0;
      if (_previousNode) {
	  odom = new MapNodeBinaryRelation(_mapManager);
	  odom->nodes()[0]=_previousNode;
	  odom->nodes()[1]=currentNode;
	  odom->setTransform(_previousNodeTransform.inverse()*currentNodeTransform);	
	  Eigen::Matrix<double, 6, 6> info;
	  info.setIdentity();
	  info = info * 100;
	  odom->setInformationMatrix(info);
	  _mapManager->addRelation(odom);
	  currentNode->setOdometry(odom);
	  currentNode->setPreviousNode(_previousNode);
      }
      put(currentNode);
      if (odom)
	put(odom);
      if (currentNode->imu()){
	_mapManager->addRelation(currentNode->imu());
	put(currentNode->imu());
      }
      _previousNode = currentNode;
      _previousNodeTransform = currentNodeTransform;
    }
  }

  BaseSensorDataNode* SyncSensorDataNodeMaker::makeNode(MapManager* manager, BaseSensorData* data) {
    SynchronizedSensorData* sdata = dynamic_cast<SynchronizedSensorData*>(data);
    if (! sdata)
      return 0;
    SyncSensorDataNode* snode = new SyncSensorDataNode(manager, sdata);
    for (size_t i = 0; i<sdata->sensorDatas.size(); i++){
      IMUData* imu = dynamic_cast<IMUData*>(sdata->sensorDatas[i]);
      if (imu){
	MapNodeUnaryRelation* imuRel = new MapNodeUnaryRelation(manager);
	imuRel->nodes()[0] = snode;
	snode->setImu(imuRel);
	break;
      }
    }
    return snode;
  }
  
  BOSS_REGISTER_CLASS(BaseSensorDataNode);
  BOSS_REGISTER_CLASS(SyncSensorDataNode);
}
