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
#include "g2o_frontend/boss_map/sensing_frame_node.h"
#include "g2o_frontend/boss_map_building/map_g2o_reflector.h"
#include "base_tracker.h"
#include "pwn_tracker.h"
#include "pwn_cloud_cache.h"
#include "pwn_closer.h"
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
  Synchronizer sync;

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
  
  std::string topic = "/camera/depth_registered/image_rect_raw";
  int scale = 4;
  sync.addSyncTopic(topic);

  Serializer ser;
  ser.setFilePath(fileout.c_str());
  
  cerr <<  "running logger with arguments: filein[" << filein << "] fileout: [" << fileout << "]" << endl;

  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  conf->serializeInternals(ser);
  ser.writeObject(*conf);
  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);

  MapManager* manager = new MapManager();
  ser.writeObject(*manager);
  

  MapG2OReflector* optimizer = new MapG2OReflector(manager);
  PwnCloudCache* cache = new PwnCloudCache(converter, conf, topic, scale, 250, 260);
  PwnCloudCacheHandler* cacheHandler = new PwnCloudCacheHandler(manager, cache);
  PwnMatcherBase* matcher = new PwnMatcherBase(aligner, converter);
  PwnTracker* tracker = new PwnTracker(matcher, cache, manager, conf);
  PwnCloser* closer = new PwnCloser(tracker);
  tracker->setScale(scale);


  std::list<Serializable*> trackerOutput;
  SensingFrameNodeMaker* nodeMaker = new SensingFrameNodeMaker();
  nodeMaker->init(manager,conf);
  StreamProcessor::PropagatorOutputHandler* sync2nm=new StreamProcessor::PropagatorOutputHandler(&sync, nodeMaker);
  MARKUSED(sync2nm);

  //boss_map_building::BaseTracker* tracker = new boss_map_building::BaseTracker(manager, conf);
  StreamProcessor::PropagatorOutputHandler* nm2t=new StreamProcessor::PropagatorOutputHandler(nodeMaker, tracker);
  MARKUSED(nm2t);

  StreamProcessor::EnqueuerOutputHandler* nm2q=new StreamProcessor::EnqueuerOutputHandler(nodeMaker, trackerOutput);
  MARKUSED(nm2q);

  
  StreamProcessor::WriterOutputHandler* writer = new StreamProcessor::WriterOutputHandler(tracker, &ser);
  tracker->init();
  
  //OdometryRelationAdder* odometryAdder = new OdometryRelationAdder(manager, conf);
  MARKUSED(writer);

  for (size_t i = 0; i< sensorDatas.size(); i++){
    BaseSensorData* data = sensorDatas[i];
    sync.process(data);
  }

  optimizer->graph()->save("out.g2o");
}
