#include <set>
#include <QGLViewer/qglviewer.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <GL/gl.h>

#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/boss_map/reference_frame_relation.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include "g2o_frontend/boss_map/laser_sensor.h"
#include "g2o_frontend/boss_map/imu_sensor.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "g2o_frontend/boss/bidirectional_serializer.h"
#include "g2o_frontend/boss_map_building/map_g2o_wrapper.h"
#include "g2o_frontend/pwn_core/pwn_static.h"
#include "g2o_frontend/pwn_boss/pwn_io.h"
// just to make the linker happy
#include "pwn_tracker.h"
#include "pwn_tracker_actions.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include "pwn_tracker_viewer.h"
#include "pwn_closer.h"

#include <QApplication>

using namespace pwn;
using namespace pwn_tracker;
using namespace boss_map;
using namespace boss;
using namespace std;

const char* banner[]={
  "pwn_tracker_boss: does SLAM on a raw boss log",
  "usage: pwn_tracker_boss <configfile> <inputFile>",
  "example: pwn_tracker_boss  integral_kinect_3m.conf myInput.log", 
  "the ouptur is written in "
  " - myInput_tracked.log (tracker output)",
  " - myInput_tracked_closed.log (tracker + closer output)",
  " - myInput_tracked.d (binary files)", 
};



void printBanner (){
  int c=0;
  while (banner[c]){
    cerr << banner [c] << endl;
    c++;
  }
}

std::string removeExtension(const std::string& filename) {
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot); 
}


int main(int argc, char** argv) {
  Deserializer des;
  
  if (argc<3){
    printBanner();
    return 0;
  }

  //bool makeVis=true; 
  bool makeVis=false;
  if (argc > 4)
    makeVis = true;

  pwn_boss::Aligner* aligner;  
  pwn_boss::DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;
    
  Serializer confSer;
  confSer.setFilePath("confout.conf");
  for (size_t i=0; i<instances.size(); i++) {
      confSer.writeObject(*instances[i]);
    }
  if (! aligner || ! converter) {
    throw std::runtime_error("AAAAAA");
  }
  des.setFilePath(argv[2]);
  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);

  std::string odom_frame_id = conf->baseReferenceFrameId();
  cerr << "base reference frame id: " << odom_frame_id << endl;
  
  std::string depth_topic = "/camera/depth_registered/image_rect_raw";
  //std::string depth_topic = "/camera/depth_registered/image_raw";
  cerr << "depth topic: " << depth_topic << endl;
  

  string outBaseFile;
  if (argc==3){
    outBaseFile = removeExtension(argv[2]);
    cerr << "baseFile: " << outBaseFile << endl;
  } else {
    outBaseFile = removeExtension(argv[3]); 
    cerr << "baseFile: " << outBaseFile << endl;
 }

  string outTrackerFile = outBaseFile+"_tracked.log";
  string outCloserFile = outBaseFile+"_tracked_closed.log";
  string outBinaryDir=outTrackerFile+".d/<classname>.<id>.<ext>";

  cerr << "tracker output: [" << outTrackerFile << "]" << endl;
  cerr << "closer output : [" << outCloserFile << "]" << endl;
  cerr << "binary dir    : [" << outBinaryDir << "]" << endl;

  Serializer ser;
  ser.setFilePath(outTrackerFile);
  ser.setBinaryPath(outBinaryDir);
  MapManager* manager = new MapManager();
  ser.writeObject(*manager);

  // install the optimization wrapper
  G2oWrapper* wrapper = new G2oWrapper(manager);
  wrapper->_selector=new PwnCloserActiveRelationSelector(manager);
  int scale = 4;
  // create a cache for the frames
  PwnCache* cache  = new PwnCache(converter, scale, 400, 410);
  PwnCacheHandler* cacheHandler = new PwnCacheHandler(manager, cache);
  manager->actionHandlers().push_back(cacheHandler);
  cacheHandler->init();


  // create a closer
  PwnCloser* closer = new PwnCloser(aligner,converter,manager,cache);
  closer->setScale(scale);
  
  // install a closure criterion to select nodes in the clser
  DistancePoseAcceptanceCriterion criterion(manager);
  criterion.setRotationalDistance(M_PI/4);
  criterion.setTranslationalDistance(1);
  closer->setCriterion(&criterion);
  closer->_debug = false;
  
  std::list<Serializable*> objects;
  PwnTracker* tracker=new PwnTracker(aligner, converter, manager, cache);
  tracker->setScale(scale);
  tracker->init();
  tracker->setNewFrameInliersFraction(0.4);

  NewFrameWriteAction* frameWriter = new NewFrameWriteAction(&ser,tracker);
  tracker->newFrameActions().push_back(frameWriter);

  NewRelationWriteAction* relationWriter = new NewRelationWriteAction(&ser,tracker);
  tracker->newRelationActions().push_back(relationWriter);

  NewFrameEnqueueAction* frameEnqueuer = new NewFrameEnqueueAction(objects,tracker);
  tracker->newFrameActions().push_back(frameEnqueuer);

  NewRelationEnqueueAction* relationEnqueuer = new NewRelationEnqueueAction(objects,tracker);
  tracker->newRelationActions().push_back(relationEnqueuer);

  NewFrameCloserAdder* closerFrameAdder = new NewFrameCloserAdder(closer, tracker);
  tracker->newFrameActions().push_back(closerFrameAdder);
  
  CloserRelationAdder* closerRelationAction = new CloserRelationAdder(objects, closer, wrapper, tracker);
  tracker->newRelationActions().push_back(closerRelationAction);
  
  VisState* visState=0;
  MyTrackerViewer *viewer = 0;
  QApplication* app =0;
  if (makeVis) {
    visState = new VisState(manager);
    NewFrameVisualizerCreator* frameVis = new NewFrameVisualizerCreator(visState, tracker);
    tracker->newFrameActions().push_back(frameVis);
    
    CloserRelationVisualizer* closerVisualizer = new CloserRelationVisualizer(closer, visState, tracker);
    tracker->newRelationActions().push_back(closerVisualizer);

    app = new QApplication(argc,argv);
    viewer = new MyTrackerViewer(visState);
    viewer->show();

  }

  
  PinholeImageData* previousImage = 0;
  Eigen::Isometry3d previousPose;
  for (size_t i=0; i<sensorDatas.size(); i++){
    PinholeImageData* imageData = dynamic_cast<PinholeImageData*>(sensorDatas[i]);
    if (imageData && imageData->topic() == depth_topic) {
      Eigen::Isometry3d pose=imageData->robotReferenceFrame()->transform();
      Eigen::Isometry3d sensorOffset_ = conf->sensorOffset(imageData->sensor());
      Eigen::Isometry3d initialGuess_ = previousPose.inverse()*pose;
      Eigen::Matrix3d cameraMatrix_ = imageData->cameraMatrix();

      
      Eigen::Isometry3f sensorOffset;
      convertScalar(sensorOffset, sensorOffset_);
      Eigen::Isometry3f initialGuess;
      convertScalar(initialGuess, initialGuess_);
      Eigen::Matrix3f cameraMatrix;
      convertScalar(cameraMatrix, cameraMatrix_);
      

      if (! previousImage){
	initialGuess.setIdentity();
      }	
      
      pwn::DepthImage depthImage;
      ImageBLOB* blob = imageData->imageBlob().get();
      //depthImage.fromCvMat(blob->cvImage());
      DepthImage_convert_16UC1_to_32FC1(depthImage, blob->cvImage());
      tracker->processFrame(depthImage, sensorOffset, cameraMatrix, initialGuess);
      delete blob;
      previousPose = pose;
      // delete previousImage
      previousImage = imageData;
      if (makeVis)
	viewer->updateGL();
    }
    if (makeVis)
      app->processEvents();
  }
  if (makeVis)
    visState->final=true;
  cerr << "done, writing out" << endl;
  ser.setFilePath(outCloserFile);
  ser.writeObject(*manager);
  for (std::list<Serializable*>::iterator it=objects.begin(); it!=objects.end(); it++){
    ser.writeObject(**it);
    cerr << ".";
  }
  cerr << "hits:  " << cache->hits() << " misses: " << cache->misses() << " hits/misses" <<
    float(cache->hits())/float(cache->hits()+cache->misses()) << endl;

  cerr << "cache: calls:  " << cache->numCalls << " time: " << cache->cumTime << 
    " t/calls: " << cache->cumTime/cache->numCalls << endl;
  cerr << "tracker: calls:  " << tracker->numCalls << " time: " << tracker->cumTime << 
    " t/calls: " << tracker->cumTime/tracker->numCalls << endl;

  if (makeVis) {
    viewer->updateGL();
    while(viewer->isVisible()){
      app->processEvents();
    }
  } 
}
