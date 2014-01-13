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

using namespace pwn_tracker;
using namespace boss_map_building;
using namespace boss_map;
using namespace boss;
using namespace std;

int main(int argc, char** argv) {
  std::string fileconf = argv[1];
  std::string filein = argv[2];
  std::string fileout = argv[3];

  StreamProcessor* sp=0;
  std::list<Serializable*> objects;
  
  
  // read the configuration ans seek for a guy called "mySLAMPipeline".
  Deserializer confDes;
  confDes.setFilePath(fileconf);
  sp=loadProcessor("mySLAMPipeline", confDes, objects);
  
  Serializable* o;
  while (( o = confDes.readObject()) )
    objects.push_back(o);
    
 
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

  // MapG2OReflector* optimizer = group->byType<MapG2OReflector>(pos);
  // if (! optimizer) {
  //   cerr << "unable to find the optimizer" << endl;
  //   return 0;
  // }

  Deserializer des;
  des.setFilePath(filein.c_str());
  
  cerr <<  "running the pipeline with arguments: filein[" << filein << "] fileout: [" << fileout << "]" << endl;

  // read the log up to the configuration file
  RobotConfiguration* conf = 0;
  while ( (o=des.readObject()) ){
    objects.push_back(o);
    RobotConfiguration * conf_ = dynamic_cast<RobotConfiguration*>(o);
    if (conf_) {
      conf = conf_;
      break;
    }
  }
  if (! conf) {
    cerr << "unable to get robot configuration, aborting " << endl;
    return -1;
  }
  
  group->setRobotConfiguration(conf);
  QApplication* app = 0;
  MyTrackerViewer *viewer = 0;

  // check if we have a gui
  PwnSLAMVisualizerProcessor* visProc = dynamic_cast<PwnSLAMVisualizerProcessor*>(group->byName("myVisState"));
  if (visProc) {
    app = new QApplication(argc,argv);
    viewer = new MyTrackerViewer(visProc->_visState);
    viewer->show();
  }

  // construct a queue handler to hold the elements that are output by the processing pipeline
  StreamProcessor::EnqueuerOutputHandler* o2q = new StreamProcessor::EnqueuerOutputHandler(group->lastNode, &objects);
  MARKUSED(o2q);

  // read the data once at a time, and see if you have to do some gui business
  Serializable* s;
  while((s=des.readObject())) {
    group->process(s);
    if (visProc && visProc->_needRedraw){
      cerr << "updateGL" << endl;
      viewer->updateGL();
      visProc->_needRedraw = false;
      app->processEvents();
    }
  }

  // write out all what the system has done
  Serializer ser;
  ser.setFilePath(fileout.c_str());
  cerr << "writing out " << objects.size() << " objects... ";
  while (! objects.empty()){
    ser.writeObject(*objects.front());
    objects.pop_front();
  }
  cerr << "done"  << endl;

  // if we have the gui provide with a nice show
  if (! visProc)
    return 0;
  else {
    visProc->_visState->final = true;
    viewer->updateGL();
    while(viewer->isVisible()){
      app->processEvents();
    }
  }

  //optimizer->graph()->save("graph.g2o");
}
