#include <list>
#include <set>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/boss_map/reference_frame_relation.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include "g2o_frontend/boss_map/laser_sensor.h"
#include "g2o_frontend/boss_map/imu_sensor.h"
#include "g2o_frontend/boss_map/sensor_data_synchronizer.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/sensor_data_node.h"
#include "g2o_frontend/boss_map_building/map_g2o_reflector.h"
#include "pwn_tracker.h"
#include "pwn_cloud_cache.h"
#include "g2o_frontend/pwn_boss/pwn_io.h"

#define MARKUSED(X)  X=X

/*#include "g2o_frontend/pwn_boss/pwn_sensor_data.h"*/

using namespace pwn_tracker;
using namespace boss_map_building;
using namespace boss_map;
using namespace boss;
using namespace std;

//pwn_boss::PWNSensorData data;

int main(int argc, char** argv) {
    
  

  // create a synchronizer
  SensorDataSynchronizer sync;
  
  std::string fileconf = argv[1];
  std::string filein = argv[2];
  std::string fileout = argv[3];
  Deserializer des;

  des.setFilePath(fileconf);
  pwn_boss::Aligner* aligner;  
  pwn_boss::DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;

  des.setFilePath(filein.c_str());
  
  int scale = 4;
 
  //std::string topic = "/kinect/depth_registered/image_raw";
  //sync.addSyncTimeCondition(topic,"/imu/data",0.05);
  std::string topic = "/camera/depth_registered/image_rect_raw";
  sync.addSyncTopic(topic);

  Serializer ser;
  ser.setFilePath(fileout.c_str());
  
  cerr <<  "running logger with arguments: filein[" << filein << "] fileout: [" << fileout << "]" << endl;


  // read the configuration file
  RobotConfiguration* conf = 0;
    std::list<Serializable*> objects;
  Serializable* o;
  while ( (o=des.readObject()) ){
    objects.push_back(o);
    RobotConfiguration * conf_ = dynamic_cast<RobotConfiguration*>(o);
    if (conf_) {
      conf = conf_;
      break;
    }
  }
  if (! conf) {
    cerr << "unable to get conf " << endl;
  }


  MapManager* manager = new MapManager();
  objects.push_back(manager);
 
  SyncSensorDataNodeMaker* nodeMaker = new SyncSensorDataNodeMaker(manager,conf);
  sync.setTopic("sync");
  nodeMaker->setTopic("sync");
  

  PwnCloudCache* cache = new PwnCloudCache(converter, conf, topic, scale, 250, 260);
  PwnCloudCacheHandler* cacheHandler = new PwnCloudCacheHandler(manager, cache);
  PwnMatcherBase* matcher = new PwnMatcherBase(aligner, converter);
  PwnTracker* tracker = new PwnTracker(matcher, cache, manager, conf);
  //tracker->setScale(scale);
  tracker->setTopic(topic);

  
  StreamProcessor::PropagatorOutputHandler* sync2nm=new StreamProcessor::PropagatorOutputHandler(&sync, nodeMaker);
  MARKUSED(sync2nm);

  StreamProcessor::PropagatorOutputHandler* nm2t=new StreamProcessor::PropagatorOutputHandler(nodeMaker, tracker);
  MARKUSED(nm2t);

  //StreamProcessor::WriterOutputHandler* writer = new StreamProcessor::WriterOutputHandler(tracker, &ser);
  tracker->init();

  
  StreamProcessor::EnqueuerOutputHandler* enqueuer = new StreamProcessor::EnqueuerOutputHandler(tracker
, &objects);
							
  

  Serializable* s;
  while((s=des.readObject())) {
    sync.process(s);
  }

  while (! objects.empty()){
    ser.writeObject(*objects.front());
    objects.pop_front();
  }


}
