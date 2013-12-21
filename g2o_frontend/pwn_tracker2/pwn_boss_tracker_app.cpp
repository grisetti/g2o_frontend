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
#include "pwn_tracker.h"
#include "pwn_cloud_cache.h"
#include "g2o_frontend/pwn_boss/pwn_io.h"

#define MARKUSED(X)  X=X

using namespace pwn_tracker;
using namespace boss_map_building;
using namespace boss_map;
using namespace boss;
using namespace std;

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
  

  PwnCloudCache* cache = new PwnCloudCache(converter, conf, topic, scale, 5, 10);
  PwnCloudCacheHandler* cacheHandler = new PwnCloudCacheHandler(manager, cache);
  cacheHandler->init();
  PwnMatcherBase* matcher = new PwnMatcherBase(aligner, converter);
  PwnTracker* tracker = new PwnTracker(matcher, cache, manager, conf);
  tracker->setTopic(topic);

  std::list<Serializable*> syncOutput;
  std::list<Serializable*> nodeMakerOutput;
  std::list<Serializable*> trackerOutput;
  std::list<Serializable*> closerOutput;
  SensingFrameNodeMaker* nodeMaker = new SensingFrameNodeMaker();
  nodeMaker->init(manager,conf);

  StreamProcessor::EnqueuerOutputHandler* sync2q=new StreamProcessor::EnqueuerOutputHandler(&sync, &syncOutput);
  MARKUSED(sync2q);

  StreamProcessor::EnqueuerOutputHandler* nm2q=new StreamProcessor::EnqueuerOutputHandler(nodeMaker, &nodeMakerOutput);
  MARKUSED(nm2q);
  
  StreamProcessor::EnqueuerOutputHandler* t2q=new StreamProcessor::EnqueuerOutputHandler(tracker, &trackerOutput);
  MARKUSED(t2q);

  StreamProcessor::WriterOutputHandler* writer = new StreamProcessor::WriterOutputHandler(tracker, &ser);
  MARKUSED(t2q);
  tracker->init();
  
  matcher->_frameInlierDepthThreshold = 50;

  MapG2OReflector * reflector = new MapG2OReflector(manager);

  for (size_t i = 0; i< sensorDatas.size(); i++){
    BaseSensorData* data = sensorDatas[i];
    sync.process(data);
    while(!syncOutput.empty()){
      Serializable* s= syncOutput.front();
      syncOutput.pop_front();
      nodeMaker->process(s);
    }

    while(!nodeMakerOutput.empty()){
      Serializable* s= nodeMakerOutput.front();
      nodeMakerOutput.pop_front();
      tracker->process(s);
    }
  }
  reflector->graph()->save("tracker_out.g2o");
}
