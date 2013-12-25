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
  //std::string topic = "/kinect/depth_registered/image_raw";
  int scale = 4;
  
  
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
    return false;
  }

  // construct the cloud cache
  PwnCloudCache* cache = new PwnCloudCache(converter, conf, topic, scale, 250, 260);
  PwnCloudCacheHandler* cacheHandler = new PwnCloudCacheHandler(manager, cache);
  cacheHandler->init();

  VisState* visState=0;
  MyTrackerViewer *viewer = 0;
  QApplication* app =0;
  
  visState = new VisState(manager);
  app = new QApplication(argc,argv);
  viewer = new MyTrackerViewer(visState);
  viewer->show();
  
  Serializable* s;
  while((s=des.readObject())) {
    cerr << ".";
    NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
    if (km) {
      PwnCloudCache::HandleType h=cache->get((SyncSensorDataNode*) km->keyNode);
      VisCloud* visCloud = new VisCloud(h.get());
      visState->cloudMap.insert(make_pair(km->keyNode, visCloud));
      viewer->updateGL();
    }
    app->processEvents();
  }

  visState->final = true;
  
  viewer->updateGL();
  while(viewer->isVisible()){
    app->processEvents();
  }
  
}
