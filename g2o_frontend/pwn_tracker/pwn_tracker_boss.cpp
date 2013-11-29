#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/boss_logger/bframerelation.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "g2o_frontend/boss_logger/blasersensor.h"
#include "g2o_frontend/boss_logger/bimusensor.h"
#include "g2o_frontend/boss_logger/brobot_configuration.h"
#include "g2o_frontend/boss/bidirectional_serializer.h"

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// just to make the linker happy
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "cache.h"
#include "pwn_tracker_g2o_wrapper.h"
#include "pwn_closer.h"


using namespace pwn;
using namespace pwn_tracker;
using namespace boss_logger;
using namespace boss;
using namespace std;


const char* banner[]={
  "boss_playback: visualizes a boss log",
  "",
  "usage: boss_playback filein",
  "example: boss_playback test.log sync_test.log", 
  "",
  "commands:", 
  "'n': moves to the next frame", 
  "'p': moves to the previous frame", 
    0
};

void printBanner (){
  int c=0;
  while (banner[c]){
    cerr << banner [c] << endl;
    c++;
  }
}


class MyTracker: public PwnTracker{
public:
  MyTracker(pwn::Aligner* aligner, 
	    pwn::DepthImageConverter* converter, 
	    boss_map::MapManager* manager,
	    PwnCache* cache,
	    PwnCloser* closer,
	    G2oWrapper* optimizer,
	    boss::Serializer* ser):
    PwnTracker(aligner, converter, manager, cache){
    objects.push_back(manager);
    this->ser = ser;
    this->closer = closer;
    this->optimizer = optimizer;
    committedRelations = 0;
    lastFrameAdded = 0;
  }
  
  virtual void newFrameCallback(PwnTrackerFrame* frame) {
    objects.push_back(frame);
    ser->writeObject(*frame);
    closer->addFrame(frame);
    lastFrameAdded=frame;
  }
  virtual void newRelationCallback(PwnTrackerRelation* relation) {
    objects.push_back(relation);

    ser->writeObject(*relation);
    closer->addRelation(relation);
    int cr = 0;
    for(std::list<PwnCloserRelation*>::iterator it=closer->committedRelations().begin();
	it!=closer->committedRelations().end(); it++){
      objects.push_back(*it);
      cr++;
    }
    if (cr>committedRelations){
      char fname[100];
      optimizer->optimize();
      sprintf(fname, "out-%05d.g2o", lastFrameAdded->seq);
      optimizer->save(fname);
    }
    cr=committedRelations;
  }
std::list<Serializable*> objects;
protected:
  boss::Serializer* ser;
  PwnCloser* closer;
  G2oWrapper* optimizer;
  PwnTrackerFrame* lastFrameAdded;
  int committedRelations;
};

int main(int argc, char** argv) {
  Deserializer des;
  
  if (argc<3){
    printBanner();
    return 0;
  }

  pwn::Aligner* aligner;  pwn::DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;
    
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
  cerr << "depth topic: " << depth_topic << endl;
  
  

  string outfile=argv[3];
  string outBinaryDir=outfile+".d/<classname>.<id>.<ext>";

  Serializer ser;
  ser.setFilePath(outfile);
  ser.setBinaryPath(outBinaryDir);
  MapManager* manager = new MapManager();
  ser.writeObject(*manager);

  // install the optimization wrapper
  G2oWrapper* wrapper = new G2oWrapper(manager);

  int scale = 4;
  // create a cache for the frames
  PwnCache* cache  = new PwnCache(converter, scale, 100);
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

  MyTracker* tracker=new MyTracker(aligner, converter, manager, cache, closer, wrapper, &ser);
  tracker->_scale = scale;
  tracker->init();

  PinholeImageData* previousImage = 0;
  Eigen::Isometry3d previousPose;
  for (size_t i=0; i<sensorDatas.size(); i++){
    PinholeImageData* imageData = dynamic_cast<PinholeImageData*>(sensorDatas[i]);
    if (imageData && imageData->topic() == depth_topic) {
      cerr << "********************* NEW FRAME *********************" << endl;
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
      depthImage.fromCvMat(blob->cvImage());
      tracker->processFrame(depthImage, sensorOffset, cameraMatrix, initialGuess);
      delete blob;
      previousPose = pose;
      // delete previousImage
      previousImage = imageData;
    }
  }

  cerr << "done, writing out" << endl;
  ser.setFilePath("out2.log");
  for (std::list<Serializable*>::iterator it=tracker->objects.begin(); it!=tracker->objects.end(); it++){
    ser.writeObject(**it);
    cerr << ".";
  }
  cerr << endl;
}
