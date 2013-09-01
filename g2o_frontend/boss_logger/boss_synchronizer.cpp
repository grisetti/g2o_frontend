#include "set"
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

int main(int argc, char** argv) {
  if (argc<3){
    cerr << "there is an error in the command line. Me wanna two args :)" << endl;
    return 0;
  }
  Deserializer des;
  des.setFilePath(argv[1]);
  Serializable *o;


  Serializer ser;
  ser.setFilePath(argv[2]);

  // create a synchronizer
  Synchronizer sync;
  // create a reframer object, that once all messages have been put together sets them to a unique frame
  Synchronizer::Reframer reframer;
  sync.addOutputHandler(&reframer);

  // create a writer object that dumps on the disk each block of synchronized objects
  Synchronizer::Writer writer(&ser);
  sync.addOutputHandler(&writer);

  // create a deleter object that polishes the memory after writing
  Synchronizer::Deleter deleter;
  sync.addOutputHandler(&deleter);

  /* old but powerful API
  // tell the topics we want to synchronize (not necessary as they are added automatically)
  SynchronizerTopicInstance* rgbSync = sync.addSyncTopic("/kinect/rgb/image_color");
  SynchronizerTopicInstance* depthSync = sync.addSyncTopic("/kinect/depth_registered/image_raw");
  SynchronizerTopicInstance* laserSync = sync.addSyncTopic("/front_scan");
  SynchronizerTopicInstance* imuSync = sync.addSyncTopic("/imu/data");
  

  // add tiem conditions between the topics
  */
  sync.addSyncTimeCondition("/kinect/rgb/image_color","/kinect/depth_registered/image_raw",0.05);
  sync.addSyncTimeCondition("/kinect/rgb/image_color", "/imu/data",0.1);
  sync.addSyncTimeCondition("/kinect/rgb/image_color", "/front_scan", 0.1);
  


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


  for (size_t i = 0; i< sensorDatas.size(); i++){
    BaseSensorData* data = sensorDatas[i];
    // every time you add something to the synchronizer, it gets massaged.
    // the output is done through the putput handlers
    sync.addSensorData(data);

  }

}
