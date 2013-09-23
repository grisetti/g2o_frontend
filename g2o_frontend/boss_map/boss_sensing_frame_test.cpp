#include <set>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/boss_logger/bframerelation.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "g2o_frontend/boss_logger/blasersensor.h"
#include "g2o_frontend/boss_logger/bimusensor.h"
#include "g2o_frontend/boss_logger/brobot_configuration.h"
#include "boss_map.h"
#include "sensing_frame_node.h"

using namespace boss_map;
using namespace boss;
using namespace std;




void processSequentialData(SensingFrameNode* previous, SensingFrameNode* current, const std::string& topic){
  cerr << "got pair of frames, processing" << endl;
  PinholeImageData* pImage = 0;
  PinholeImageData* cImage = 0;
  for (size_t i = 0 ;  i< previous->sensorDatas().size(); i++){
    BaseSensorData* sdata = previous->sensorDatas()[i];
    if (sdata->topic()==topic){
      pImage=dynamic_cast<PinholeImageData*>(sdata);
    }
  }

  for (size_t i = 0 ;  i< current->sensorDatas().size(); i++){
    BaseSensorData* sdata = current->sensorDatas()[i];
    if (sdata->topic()==topic){
      cImage=dynamic_cast<PinholeImageData*>(sdata);
    }
  }

  std::set<MapNodeRelation*>& pRelations = previous->manager()->nodeRelations(previous);
  std::set<MapNodeRelation*>& cRelations = current->manager()->nodeRelations(current);
  MapNodeBinaryRelation* odometry = 0;
  MapNodeUnaryRelation* imu1 = 0, *imu2 = 0;
  for (std::set<MapNodeRelation*>::iterator it=pRelations.begin(); it!=pRelations.end(); it++){
    MapNodeRelation* rel=*it;
    MapNodeUnaryRelation* unary=dynamic_cast<MapNodeUnaryRelation*>(rel);
    MapNodeBinaryRelation* binary=dynamic_cast<MapNodeBinaryRelation*>(rel);
    if (unary)
      imu1 = unary;
    if (binary && binary->nodes()[1]==current)
      odometry = binary;
  }

  for (std::set<MapNodeRelation*>::iterator it=cRelations.begin(); it!=cRelations.end(); it++){
    MapNodeRelation* rel=*it;
    MapNodeUnaryRelation* unary=dynamic_cast<MapNodeUnaryRelation*>(rel);
    MapNodeBinaryRelation* binary=dynamic_cast<MapNodeBinaryRelation*>(rel);
    if (unary)
      imu2 = unary;
    if (binary && binary->nodes()[0]==previous)
      odometry = binary;
  }
  
cerr << "img1: " << pImage << " img2: " << cImage << " imu1: " << imu1 << " imu2: " << imu2 << " odometry: " << odometry << endl;
}

int main(int argc, char** argv) {
  Deserializer des;
  if (argc < 2) {
    cerr << " usage: " << argv[0] << " <input file> < output file> " << endl;
  }
  des.setFilePath(argv[1]);

  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);

  std::vector<Serializable*> newObjects;

  ReferenceFrame* currentReferenceFrame = 0;
  ReferenceFrame* previousReferenceFrame = 0;
  SensingFrameNode* previousSensingFrameNode = 0;
  SensingFrameNode* currentSensingFrameNode = 0;
  size_t i=0;
  std::vector<MapNodeRelation*> relations;
  MapManager* manager = new MapManager;
  newObjects.push_back(manager);
  std::vector<SensingFrameNode*> sensingFrames;
  while (i<sensorDatas.size()){
    bool newFrameAdded = 0;
    BaseSensorData* data =sensorDatas[i];
    if(data->robotReferenceFrame() == currentReferenceFrame){
      //cerr << "pushing data to node" << endl;
      currentSensingFrameNode->sensorDatas().push_back(data);
    } else {
      currentSensingFrameNode = new SensingFrameNode(manager);
      cerr << "creating new sensing frame (" << currentSensingFrameNode << ")" << endl;
      manager->addNode(currentSensingFrameNode);
      newFrameAdded=true;
      sensingFrames.push_back(currentSensingFrameNode);
      newObjects.push_back(currentSensingFrameNode);
      //cerr << "pushing data to node" << endl;
      currentSensingFrameNode->sensorDatas().push_back(data);
      IMUData* imuData = dynamic_cast<IMUData*>(data);
      if (imuData){
	MapNodeUnaryRelation* rel = new MapNodeUnaryRelation(manager);
	rel->setGenerator(imuData);
	newObjects.push_back(rel);
	rel->nodes()[0]=currentSensingFrameNode; 
	Eigen::Matrix<double,6,6> info;
	info.setZero();
	info.block<3,3>(3,3)=imuData->orientationCovariance().inverse();
	rel->setInformationMatrix(info);
	
	Eigen::Isometry3d iso;
	iso.linear()  = imuData->orientation().matrix();
	iso.translation().setZero();
	rel->setTransform(iso);
	manager->addRelation(rel);
     }
	
      if (previousSensingFrameNode && previousSensingFrameNode!=currentSensingFrameNode){
      	//cerr << "creating new relation";
      	MapNodeBinaryRelation* rel = new MapNodeBinaryRelation(manager);
	newObjects.push_back(rel);
      	rel->nodes()[0]=previousSensingFrameNode;
      	rel->nodes()[1]=currentSensingFrameNode;
      	rel->setTransform(previousReferenceFrame->transform().inverse()*currentReferenceFrame->transform());
	rel->setInformationMatrix(Eigen::Matrix<double,6,6>::Identity());
      	//cerr << "adding relation to manager";
      	manager->addRelation(rel);
      }
    }
    previousSensingFrameNode  = currentSensingFrameNode;
    previousReferenceFrame = currentReferenceFrame;
    currentReferenceFrame = data->robotReferenceFrame();
    if (newFrameAdded && sensingFrames.size()>3) {
      int previousIndex = sensingFrames.size()-3;
      int currentIndex = sensingFrames.size()-2;
      processSequentialData(sensingFrames[previousIndex], sensingFrames[currentIndex], "/kinect/depth_registered/image_raw");
    }
    i++;
  }


  
  // write back the log
  Serializer ser;
  ser.setFilePath(argv[2]);

  conf->serializeInternals(ser);
  ser.writeObject(*conf);

  previousReferenceFrame = 0;
  for (size_t i = 0; i< sensorDatas.size(); i++){
    BaseSensorData* data = sensorDatas[i];
    if (previousReferenceFrame!=data->robotReferenceFrame()){
      ser.writeObject(*data->robotReferenceFrame());
    }
    ser.writeObject(*data);
    previousReferenceFrame = data->robotReferenceFrame();
  }
  
  for(size_t i=0; i<newObjects.size(); i++){
    ser.writeObject(*newObjects[i]);
  }
  
  
}
