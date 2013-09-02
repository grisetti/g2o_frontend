#include <list>
#include <set>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"
#include "bimagesensor.h"
#include "blasersensor.h"
#include "bimusensor.h"
#include "bsynchronizer.h"




using namespace boss;
using namespace std;

StringSensorMap sensors;
StringFrameMap  frames;
std::vector<boss::Serializable*> objects;
std::vector<BaseSensorData*> sensorDatas;

struct TSCompare{
  bool operator()(const BaseSensorData* a, const BaseSensorData*b){
    return a->timestamp()<b->timestamp();
  }
};



const char* banner[]={
  "boss_path_generator: adds the odometry constraint between consecutive frames of  atime-ordered log",
  "",
  "usage: boss_path_generator filein fileout",
  "example: boss_path_generator test_sync.log test_sync_odom.log", 0
};

void printBanner (){
  int c=0;
  while (banner[c]){
    cerr << banner [c] << endl;
    c++;
  }
}


int main(int argc, char** argv) {
  if (argc < 3) {
    printBanner();
    return 0;
  }
  std::string filein = argv[1];
  std::string fileout = argv[2];



  Deserializer des;
  des.setFilePath(filein.c_str());

  Serializer ser;
  ser.setFilePath(fileout.c_str());
  
  cerr <<  "running path generator  with arguments: filein[" << filein << "] fileout: [" << fileout << "]" << endl;


  Serializable *o;
  while( (o=des.readObject()) ){
    cerr << ".";
    BaseSensor* sensor= dynamic_cast<BaseSensor*>(o);
    if (sensor) {
      sensors.insert(make_pair(sensor->topic(), sensor));
    }

    Frame* frame=dynamic_cast<Frame*>(o);
    if (frame && frame->name()!=""){
      frames.insert(make_pair(frame->name(), frame));
    }
    
    BaseSensorData* sensorData=dynamic_cast<BaseSensorData*>(o);
    if (sensorData){
      sensorDatas.push_back(sensorData);
    }
    objects.push_back(o);
  }
  cerr << "read: " << objects.size() << " objects"  << endl;
  cerr << "# frames: " << frames.size() << endl;
  cerr << "# sensors: " << sensors.size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;
  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);

  for(StringFrameMap::iterator it = frames.begin(); it!=frames.end(); it++)
    ser.writeObject(*(it->second));

  for(StringSensorMap::iterator it = sensors.begin(); it!=sensors.end(); it++)
    ser.writeObject(*(it->second));


  BaseSensorData* previousData = 0;
  for (size_t i = 0; i< sensorDatas.size(); i++){
    BaseSensorData* data = sensorDatas[i];
    ser.writeObject(*data->robotFrame());
    ser.writeObject(*data);
    if (previousData){
      Frame* from = previousData->robotFrame();
      Frame* to = data->robotFrame();
      FrameRelation* rel = new FrameRelation;
      rel->setFromFrame(from);
      rel->setToFrame(to);
      rel->setTransform(from->transform().inverse() * to->transform());
      rel->setInformationMatrix(Eigen::Matrix<double, 6,6>::Identity());
      ser.writeObject(*rel);
    }
  }

}
