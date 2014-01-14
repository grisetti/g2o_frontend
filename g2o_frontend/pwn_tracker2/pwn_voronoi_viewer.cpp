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
#include "manifold_voronoi_extractor.h"


#include <QApplication>

#define MARKUSED(X)  X=X

/*#include "g2o_frontend/pwn_boss/pwn_sensor_data.h"*/
using namespace manifold_voronoi;
using namespace pwn_tracker;
using namespace boss_map_building;
using namespace boss_map;
using namespace boss;
using namespace std;

int main(int argc, char** argv) {


  std::string filein = argv[1];

  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(filein);
  StreamProcessor* sp=loadProcessor("mySLAMPipeline", des, objects);
  
  if (! sp){
    cerr << "object not found, aborting";
    return 0;
  }
  
  StreamProcessorGroup* group = dynamic_cast<StreamProcessorGroup*>(sp);
  if (! group) {
    cerr << "object is not a pipeline, aborting";
    return 0;
  }
  
  // retrieve the manager from the pipeline. You will have to copy it in the result
  size_t pos = 0;
  MapManager* manager = group->byType<MapManager>(pos);
  if (! manager) {
    cerr << "unable to find the manager" << endl;
    return 0;
  }

  PwnCloudCache* cache = group->byType<PwnCloudCache>(pos);
  if (! cache) {
    cerr << "unable to find the cache" << endl;
    return 0;
  }

  cerr << "algo config loaded" << endl;

  RobotConfiguration* conf = 0;
  Serializable* s;
  while ( (s=des.readObject()) ){
    cerr << s->className() << endl;
    objects.push_back(s);
    RobotConfiguration * conf_ = dynamic_cast<RobotConfiguration*>(s);
    if (conf_) {
      conf = conf_;
      break;
    }
  }
  if (! conf) {
    cerr << "unable to get robot configuration, aborting " << endl;
    cerr << "objects.size(): " << objects.size() << endl;
    return -1;
  }
  cerr << "robot config loaded" << endl;

  group->setRobotConfiguration(conf);

  VisState* visState = new VisState(manager);
  QApplication* app = new QApplication(argc,argv);
  MyTrackerViewer *viewer = new MyTrackerViewer(visState);
  viewer->show();
  while((s=des.readObject())) {
    objects.push_back(s);
    NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
    if (km) {
      PwnCloudCache::HandleType h=cache->get((SyncSensorDataNode*) km->keyNode);
      VisCloud* visCloud = new VisCloud(h.get());
      visState->cloudMap.insert(make_pair(km->keyNode, visCloud));
      viewer->updateGL();
    }
    ManifoldVoronoiData* vdata = dynamic_cast<ManifoldVoronoiData*>(s);
    if (vdata) {
      ImageBLOB* blobbe = vdata->imageBlob().get();
      cv::Mat matte = blobbe->cvImage();
      MapNode* mnode = vdata->node;
      Eigen::Isometry3d iso = mnode->transform();
      cerr << " ci ho lo blobbe che lo ha preso in" << iso.translation().transpose() << endl;
      delete blobbe;
    }
    app->processEvents();
  }

  visState->final = true;
  
  viewer->updateGL();
  while(viewer->isVisible()){
    app->processEvents();
  }
  
}
