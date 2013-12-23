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
#include "pwn_closer.h"
#include "g2o_frontend/pwn_boss/pwn_io.h"
#include "pwn_tracker_viewer.h"

#include <QApplication>

#define MARKUSED(X)  X=X

/*#include "g2o_frontend/pwn_boss/pwn_sensor_data.h"*/

using namespace pwn_tracker;
using namespace boss_map_building;
using namespace boss_map;
using namespace boss;
using namespace std;

int main(int argc, char** argv) {
    
  


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
  
  // construct the synchronizer and its queue
  SensorDataSynchronizer* sync = new SensorDataSynchronizer();
  sync->addSyncTopic(topic);
  std::list<Serializable*> syncOutput;
  StreamProcessor::EnqueuerOutputHandler* sync2q=new StreamProcessor::EnqueuerOutputHandler(sync, &syncOutput);
  MARKUSED(sync2q);

  // construct the nodeMaker and its queue
  SyncSensorDataNodeMaker* nodeMaker = new SyncSensorDataNodeMaker(manager,conf);
  nodeMaker->setTopic(sync->topic());
  std::list<Serializable*> nodeMakerOutput;
  StreamProcessor::EnqueuerOutputHandler* nm2q=new StreamProcessor::EnqueuerOutputHandler(nodeMaker, &nodeMakerOutput);
  MARKUSED(nm2q);
 
  // construct the cloud cache
  PwnCloudCache* cache = new PwnCloudCache(converter, conf, topic, scale, 250, 260);
  PwnCloudCacheHandler* cacheHandler = new PwnCloudCacheHandler(manager, cache);
  cacheHandler->init();

  // construct the optimizer bound to the map
  MapG2OReflector* optimizer = new MapG2OReflector(manager);
  
  // construct the matcher object to be shared between the tracker and the closer
  PwnMatcherBase* matcher = new PwnMatcherBase(aligner, converter);
  matcher->_frameInlierDepthThreshold = 50;

  // construct the tracker object, and its queue
  PwnTracker* tracker = new PwnTracker(matcher, cache, manager, conf);
  tracker->setTopic(topic);
  std::list<Serializable*> trackerOutput;
  StreamProcessor::EnqueuerOutputHandler* t2q=new StreamProcessor::EnqueuerOutputHandler(tracker, &trackerOutput);
  MARKUSED(t2q);

  // construct the closer object and its queue
  PwnCloser* closer = new PwnCloser(tracker);
  std::list<Serializable*> closerOutput;
  StreamProcessor::EnqueuerOutputHandler* c2q=new StreamProcessor::EnqueuerOutputHandler(closer, &closerOutput);
  MARKUSED(c2q);

  DistancePoseAcceptanceCriterion* distanceCriterion = new DistancePoseAcceptanceCriterion(manager);
  distanceCriterion->setRotationalDistance(M_PI/4);
  distanceCriterion->setTranslationalDistance(1);
  KeyNodeAcceptanceCriterion* criterion = new KeyNodeAcceptanceCriterion(closer,manager, distanceCriterion);
  closer->setCriterion(criterion);
  MapCloserActiveRelationSelector* optSelector = new MapCloserActiveRelationSelector(closer, manager);

  optimizer->setSelector(optSelector);
  closer->setSelector(optSelector);
  closer->autoProcess = false;

  tracker->init();
  tracker->setScale(scale);

  Serializable* s;
  objects.push_back(manager);
  std::list<Serializable*> closerInput;


  bool makeVis = false;
  VisState* visState=0;
  MyTrackerViewer *viewer = 0;
  QApplication* app =0;
  if (makeVis) {
    visState = new VisState(manager);
    visState->relationSelector = optSelector;
    app = new QApplication(argc,argv);
    viewer = new MyTrackerViewer(visState);
    viewer->show();
  }

  
  while((s=des.readObject())) {
    sync->process(s);

    while(!syncOutput.empty()){
      Serializable* s= syncOutput.front();
      syncOutput.pop_front();
      nodeMaker->process(s);
    }
  }

  while(!nodeMakerOutput.empty()){
    Serializable* s= nodeMakerOutput.front();
    nodeMakerOutput.pop_front();
    tracker->process(s);
    
    while (!trackerOutput.empty()){
      Serializable* s = trackerOutput.front();
      trackerOutput.pop_front();
      closer->process(s);
      NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
      if (km) {
	if (makeVis) {
	  PwnCloudCache::HandleType h=cache->get((SyncSensorDataNode*) km->keyNode);
	  VisCloud* visCloud = new VisCloud(h.get());
	  visState->cloudMap.insert(make_pair(km->keyNode, visCloud));
	  
	  visState->candidateRelations=closer->candidateRelations();
	  visState->partitions = closer->partitions();
	  for (size_t i =0; i<closer->partitions().size(); i++){
	    if (&closer->partitions()[i]==closer->currentPartition()){
	      visState->currentPartitionIndex = i;
	    }
	  }
	}
	if (makeVis)
	  viewer->updateGL();
      }
    }

    bool optimize = false;
    while(!closerOutput.empty()){
      Serializable* s= closerOutput.front();
      closerOutput.pop_front();
      objects.push_back(s);
      PwnCloserRelation* rel = dynamic_cast<PwnCloserRelation*>(s);
      if (rel) {
	optimize=true;
      }
    }

    if (optimize){
      cerr<< "OPTIMIZE" << endl;
      optimizer->optimize();
    }
    if (makeVis)
      app->processEvents();
  }

  cerr << "writing out " << objects.size() << " objects" << endl;
  while (! objects.empty()){
    ser.writeObject(*objects.front());
    objects.pop_front();
  }

  optimizer->graph()->save("slam_out.g2o");
  visState->final = true;

  if (makeVis) {
    viewer->updateGL();
    while(viewer->isVisible()){
      app->processEvents();
    }
  }
  
}
