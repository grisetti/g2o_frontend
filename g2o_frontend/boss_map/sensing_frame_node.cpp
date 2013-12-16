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
  }
  
  void SensingFrameNode::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    ArrayData* adata = new ArrayData;
    for (size_t i=0; i<_sensorDatas.size(); i++){
      adata->add(new PointerData(_sensorDatas[i]));
    }
    data.setField("sensorDatas", adata);
  }

  void SensingFrameNode::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
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
    _currentSensingFrameNode = 0;
    _seq = 0;
  }

  void SensingFrameNodeMaker::init(MapManager* manager_, RobotConfiguration* config_){
    _mapManager = manager_;
    _config = config_;
    _currentSensingFrameNode = 0;
    _previousData = 0;
  }

  void SensingFrameNodeMaker::process(Serializable* s){
    put(s);
    BaseSensorData* sdata = dynamic_cast<BaseSensorData*>(s);
    if (sdata) {
      SensingFrameNode* n = processData(sdata);
      if (n)
	put(n);
    }
  }

  SensingFrameNode* SensingFrameNodeMaker::processData(BaseSensorData* data){
    SensingFrameNode * returned = 0;
    if (!_previousData || _previousData->robotReferenceFrame() != data->robotReferenceFrame()){
      // new buddy, push the old one
      returned = _currentSensingFrameNode;
      _currentSensingFrameNode = new SensingFrameNode(_mapManager);
      _currentSensingFrameNode->setSeq(_seq++);
      _currentSensingFrameNode->setTransform(data->robotReferenceFrame()->transform());
      _mapManager->addNode(_currentSensingFrameNode);
      //cerr << "New Frame created" << _currentSensingFrameNode << endl;
    }
    //cerr << "Payload added" << data->className() << endl;
    _currentSensingFrameNode->sensorDatas().push_back(data);
    _previousData = data;
    return returned;
  }

  BOSS_REGISTER_CLASS(SensingFrameNode);

}
