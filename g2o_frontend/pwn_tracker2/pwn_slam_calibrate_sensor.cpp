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

  PwnTracker* tracker = dynamic_cast<PwnTracker*>(group->byName("myTracker"));
  if (! tracker) {
    cerr << "unable to find the tracker" << endl;
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

  // now retrieve the sensor attached to the topic;
  std::string topic = tracker->topic();
  cerr << "tracker topic: " << topic << endl;
  BaseSensor* sensor = conf->sensor(topic);
  cerr << "sensor: " << sensor << endl;
  PinholeImageSensor* pinholeSensor = dynamic_cast<PinholeImageSensor*>(sensor);
  Eigen::Isometry3d T =conf->sensorOffset(pinholeSensor);
  Eigen::Isometry3d iT = T.inverse();
  cerr << "sensor position on the robot:" << t2v(T).transpose() << endl;

  group->setRobotConfiguration(conf);
  ofstream os("calib.dat");
  os << "# initial transform: " << t2v(T).transpose() << endl;
  os << "# measurements, each line is: " << endl;
  os << "# ox oy oz qox qoy qoz sx sy sz qsx qsy qsz " << endl;
  while((s=des.readObject())) {
    objects.push_back(s);
    PwnTrackerRelation* rel = dynamic_cast<PwnTrackerRelation*>(s);
    if (rel) {
      Eigen::Isometry3d t1, t2;
      {
	SyncSensorDataNode* n1 = dynamic_cast<SyncSensorDataNode*>(rel->nodes()[0]);
	SynchronizedSensorData* data = n1->sensorData();
	PinholeImageData* bdata = data->sensorData<PinholeImageData>(topic);
	ReferenceFrame* f = bdata->robotReferenceFrame();
	t1=f->transform();
      }

      {
	SyncSensorDataNode* n1 = dynamic_cast<SyncSensorDataNode*>(rel->nodes()[1]);
	SynchronizedSensorData* data = n1->sensorData();
	PinholeImageData* bdata = data->sensorData<PinholeImageData>(topic);
	ReferenceFrame* f = bdata->robotReferenceFrame();
	t2=f->transform();
      }
      
      Eigen::Isometry3d zOdom = t1.inverse()*t2;
      Eigen::Isometry3d zSensor = rel->transform();
      os << t2v(zOdom).transpose() << " "
	 << t2v(iT*zSensor*T).transpose() << endl;
    }
  }
}
