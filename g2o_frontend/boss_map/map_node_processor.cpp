#include "map_node_processor.h"

namespace boss_map{
  using namespace boss;
    MapNodeProcessor::MapNodeProcessor(MapManager* manager_,   RobotConfiguration* config_) {
      _manager = manager_;
      _config = config_;
    }



  ImuRelationAdder::ImuRelationAdder(MapManager* manager_,   RobotConfiguration* config_): 
    MapNodeProcessor(manager_,config_){}
    
  void ImuRelationAdder::processNode(MapNode* node_){
    SensingFrameNode* f = dynamic_cast<SensingFrameNode*>(node_);
    if (!f)
      return;
    for (size_t i = 0; i<f->sensorDatas().size(); i++){
      BaseSensorData* s = f->sensorDatas()[i];
      IMUData* imu = dynamic_cast<IMUData*>(s);
      if (! imu)
	continue;
      
      MapNodeUnaryRelation* rel = new MapNodeUnaryRelation(_manager);
      rel->setGenerator(imu);
      rel->nodes()[0]=f; 
      Eigen::Matrix<double,6,6> info;
      info.setZero();
      //info.block<3,3>(3,3)=imu->orientationCovariance().inverse()*1e9;
      info.block<3,3>(3,3).setIdentity();
      info.block<3,3>(3,3)*=1e6;
      rel->setInformationMatrix(info);
	
      Eigen::Isometry3d iso;
      iso.linear()  = imu->orientation().matrix();
      iso.translation().setZero();
      rel->setTransform(iso);
      cerr << "Imu added (" << f << ")" << endl;
      _manager->addRelation(rel);
    }
  }

  OdometryRelationAdder::OdometryRelationAdder(MapManager* manager_,   RobotConfiguration* config_): MapNodeProcessor(manager_,config_){
    _previousNode = 0;
  }
  
  void OdometryRelationAdder::processNode(MapNode* node_){
    if (_previousNode){
      MapNodeBinaryRelation* rel = new MapNodeBinaryRelation(_manager);
      rel->nodes()[0]=_previousNode;
      rel->nodes()[1]=node_;
      rel->setTransform(_previousNode->transform().inverse()*node_->transform());
      Eigen::Matrix<double,6,6> info;
      info.setIdentity();
      info.block<3,3>(0,0)*=100;
      info.block<3,3>(3,3)*=1000;

      rel->setInformationMatrix(Eigen::Matrix<double,6,6>::Identity());
      _manager->addRelation(rel);
      cerr << "Odom added (" << _previousNode << ", " << node_ << ")" << endl;
      
    }
    _previousNode = node_;
  }

}
