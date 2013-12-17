#include "sensing_frame_node.h"
#include "map_manager.h"
#include <stdexcept>
#include <iostream>

namespace boss_map {
  using namespace std;
  using namespace boss;
  /***************************************** SensingFrame *****************************************/

  SensingFrameNode::SensingFrameNode (MapManager* manager, int id, IdContext* context):
    MapNode(manager, id, context){
    _odometry = 0;
    _imu = 0;
    _previousNode = 0;
  }
  
  void SensingFrameNode::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("imu",_imu);
    data.setPointer("odometry", _odometry);
    data.setPointer("previousNode", _previousNode);
    ArrayData* adata = new ArrayData;
    for (size_t i=0; i<_sensorDatas.size(); i++){
      adata->add(new PointerData(_sensorDatas[i]));
    }
    data.setField("sensorDatas", adata);
  }

  void SensingFrameNode::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
    data.getReference("imu").bind(_imu);
    data.getReference("odometry").bind(_odometry);
    data.getReference("previousNode").bind(_previousNode);
    ArrayData* adata = dynamic_cast<ArrayData*>(data.getField("sensorDatas"));
    _sensorDatas.resize(adata->size());
    for(size_t i = 0; i<adata->size(); i++){
      (*adata)[i].getReference().bind(_sensorDatas[i]);
    }
  }

  ReferenceFrame* SensingFrameNode::robotFrame(){
    if (! _sensorDatas.size())
      return 0;
    return _sensorDatas[0]->robotReferenceFrame();
  }


  SensingFrameNodeMaker::SensingFrameNodeMaker(){
    _mapManager = 0;
    _config = 0;
    _currentNode = 0;
    _seq = 0;
  }

  void SensingFrameNodeMaker::init(MapManager* manager_, RobotConfiguration* config_){
    _mapManager = manager_;
    _config = config_;
    _currentNode = 0;
    _previousData = 0;
    _previousNode = 0;
    _lastImu = 0;
    _lastOdom = 0;
  }

  void SensingFrameNodeMaker::process(Serializable* s){
    put(s);
    BaseSensorData* data = dynamic_cast<BaseSensorData*>(s);
    if (data) {
      SensingFrameNode * returned = 0;
      if (!_previousData || _previousData->robotReferenceFrame() != data->robotReferenceFrame()){
	// new buddy, push the old one
	returned = _currentNode;
	_currentNode = new SensingFrameNode(_mapManager);
	_currentNode->setSeq(_seq++);
	_currentNode->setTransform(data->robotReferenceFrame()->transform());
	_mapManager->addNode(_currentNode);
	if (_lastImu) {
	  _lastImu->nodes()[0] = returned;
	}
	if (returned) {
	  returned->setOdometry(_lastOdom);
	  put(returned);
	  if (_lastImu) {
	    put(_lastImu);
	  }
	  if (_previousNode) {
	    MapNodeBinaryRelation* rel = new MapNodeBinaryRelation(_mapManager);
	    rel->nodes()[0]=_previousNode;
	    rel->nodes()[1]=returned;
	    rel->setTransform(_previousNode->transform().inverse()*returned->transform());
	    rel->setInformationMatrix(Eigen::Matrix<double, 6, 6>::Identity());
	    _mapManager->addRelation(rel);
	    put(rel);
	    _lastOdom=rel;
	  }
	}
	_lastImu = 0;
	//cerr << "New Frame created" << _currentNode << endl;
	_previousNode = returned;
      }
      //cerr << "Payload added" << data->className() << endl;
      IMUData* imuData = dynamic_cast<IMUData*>(data);
      if (imuData) {
	MapNodeUnaryRelation * imu = new MapNodeUnaryRelation(_mapManager);
	imu->setTransform(imu->transform());
	imu->setInformationMatrix(Eigen::Matrix<double, 6, 6>::Identity());
	_lastImu = imu;
      }
      _currentNode->sensorDatas().push_back(data);
      _previousData = data;
    }
    
  }

  SensingFrameNode* SensingFrameNodeMaker::processData(BaseSensorData* data){
    SensingFrameNode * returned = 0;
    if (!_previousData || _previousData->robotReferenceFrame() != data->robotReferenceFrame()){
      // new buddy, push the old one
      returned = _currentNode;
      _currentNode = new SensingFrameNode(_mapManager);
      _currentNode->setSeq(_seq++);
      _currentNode->setTransform(data->robotReferenceFrame()->transform());
      _mapManager->addNode(_currentNode);
      //cerr << "New Frame created" << _currentNode << endl;
    }
    //cerr << "Payload added" << data->className() << endl;
    _currentNode->sensorDatas().push_back(data);
    _previousData = data;
    return returned;
  }

  BOSS_REGISTER_CLASS(SensingFrameNode);

}
