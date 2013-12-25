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
#include "g2o_frontend/boss_map_building/map_g2o_reflector.h"
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
  MapManager* manager = 0;
  std::list<Serializable*> objects;
  Serializable* o;
  while ( (o=des.readObject()) ){
    objects.push_back(o);
    RobotConfiguration * conf_ = dynamic_cast<RobotConfiguration*>(o);
    if (conf_)
      conf = conf_;
    MapManager * manager_ = dynamic_cast<MapManager*>(o);
    if (manager_){
      manager = manager_;
    }
    if (conf && manager)
      break;
  }
  if (! conf || !manager) {
    cerr << "unable to get conf or manager" << endl;
  }

  
  MapG2OReflector* optimizer = new MapG2OReflector(manager);
  PwnCloudCache* cache = new PwnCloudCache(converter, conf, topic, scale, 250, 260);
  PwnCloudCacheHandler* cacheHandler = new PwnCloudCacheHandler(manager, cache);
  cacheHandler->init();
  
  
  PwnMatcherBase* matcher = new PwnMatcherBase(aligner, converter);
  PwnTracker* tracker = new PwnTracker(matcher, cache, manager, conf);
  tracker->setTopic("sync");

  PwnCloser* closer = new PwnCloser(tracker);
  DistancePoseAcceptanceCriterion* distanceCriterion = new DistancePoseAcceptanceCriterion(manager);
  distanceCriterion->setRotationalDistance(M_PI/4);
  distanceCriterion->setTranslationalDistance(1);
  KeyNodeAcceptanceCriterion* criterion = new KeyNodeAcceptanceCriterion(closer,manager, distanceCriterion);
  closer->setCriterion(criterion);
  MapCloserActiveRelationSelector* optSelector = new MapCloserActiveRelationSelector(closer, manager);

  optimizer->setSelector(optSelector);
  closer->setSelector(optSelector);
  tracker->init();
  tracker->setScale(scale);
  

  std::list<Serializable*> closerOutput;
  StreamProcessor::EnqueuerOutputHandler* c2q=new StreamProcessor::EnqueuerOutputHandler(closer, &closerOutput);
  MARKUSED(c2q);
  
  
  //OdometryRelationAdder* odometryAdder = new OdometryRelationAdder(manager, conf);
  //MARKUSED(writer);
  matcher->_frameInlierDepthThreshold = 50;
  closer->autoProcess = false;

  MapNode * _lastNodeAdded = 0;
  Serializable* s;
  bool hasToProcess = false;
  while((s=des.readObject())) {
    objects.push_back(s);
    NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
    if (km) {
      if (_lastNodeAdded)
	hasToProcess = true;
      closer->addKeyNode(km->keyNode);
      _lastNodeAdded = km->keyNode;
    }
    if (hasToProcess) {
      closer->MapCloser::process();
      closer->flush();
      hasToProcess = false;
    }
    bool optimize = false;
    while(!closerOutput.empty()){
      Serializable* s= closerOutput.front();
      objects.push_back(s);
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
  }
  /*
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
  */
  optimizer->graph()->save("closer_out.g2o");
}
