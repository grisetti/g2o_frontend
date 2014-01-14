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

/*
class KeyNodeAcceptanceCriterion: public DistancePoseAcceptanceCriterion{
public:
  KeyNodeAcceptanceCriterion(MapCloser* closer_, MapManager* manager_) : DistancePoseAcceptanceCriterion(manager_){
    _closer= closer_;
  }
  virtual bool accept(MapNode* n) {
    if (_closer->_trackerFrames.find(n->seq())==_closer->_trackerFrames.end())
      return false;
    return DistancePoseAcceptanceCriterion::accept(n);
  }
protected:
  MapCloser* _closer;
};
*/

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
  cacheHandler->init();
  PwnMatcherBase* matcher = new PwnMatcherBase(aligner, converter);
  PwnTracker* tracker = new PwnTracker(matcher, cache, manager, conf);
  tracker->setTopic(topic);
  PwnCloser* closer = new PwnCloser(tracker);
  DistancePoseAcceptanceCriterion* distanceCriterion = new DistancePoseAcceptanceCriterion(manager);
  distanceCriterion->setRotationalDistance(M_PI/4);
  distanceCriterion->setTranslationalDistance(1);
  KeyNodeAcceptanceCriterion* criterion = new KeyNodeAcceptanceCriterion(closer,manager, distanceCriterion);
  closer->setCriterion(criterion);

  MapCloserActiveRelationSelector* optSelector = new MapCloserActiveRelationSelector(manager);

  optimizer->setSelector(optSelector);
  closer->setSelector(optSelector);
  tracker->init();
  tracker->setScale(scale);
  



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

  StreamProcessor::EnqueuerOutputHandler* c2q=new StreamProcessor::EnqueuerOutputHandler(closer, &closerOutput);
  MARKUSED(c2q);
  
  //StreamProcessor::WriterOutputHandler* writer = new StreamProcessor::WriterOutputHandler(tracker, &ser);
  tracker->init();
  
  //OdometryRelationAdder* odometryAdder = new OdometryRelationAdder(manager, conf);
  //MARKUSED(writer);
  matcher->_frameInlierDepthThreshold = 50;
  closer->autoProcess = false;

  MapNode * _lastNodeAdded = 0;
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

    //bool hasToProcess = false;
    while(!trackerOutput.empty()){
      Serializable* s= trackerOutput.front();
      ser.writeObject(*s);
      trackerOutput.pop_front();
      /*
      closer->process(s);
      NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
      if (km) {
	if (_lastNodeAdded)
	  hasToProcess = true;
	_lastNodeAdded = km->keyNode;
      }
      */
    }
    //if (hasToProcess)
    //closer->process();
    
    /*    
    bool optimize = false;
    while(!closerOutput.empty()){
      Serializable* s= closerOutput.front();
      closerOutput.pop_front();
      PwnCloserRelation* rel = dynamic_cast<PwnCloserRelation*>(s);
      if (rel) {
	optimize=true;
      }
    }
    if (optimize){
      cerr<< "OPTIMIZE" << endl;
      optimizer->optimize();
    }
    */
  }
  
  std::list<MapNodeRelation*> outliers;
  for(std::set<MapNodeRelation*>::iterator it=manager->relations().begin(); it!=manager->relations().end(); it++){
    ClosureInfo* info = dynamic_cast<ClosureInfo*>(*it);
    if (info && !info->accepted)
      outliers.push_back(*it);
  }
  for (std::list<MapNodeRelation*>::iterator it = outliers.begin(); it!=outliers.end(); it++){
    manager->removeRelation(*it);
  }
  cerr << "erasing " << outliers.size() << " outliers from the g2o output" << endl;
  optimizer->graph()->save("out.g2o");
}
