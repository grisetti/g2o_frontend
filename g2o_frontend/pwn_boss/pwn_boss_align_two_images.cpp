#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn_core/frame.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/pwn_core/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/boss_map/bframe.h"
#include "g2o_frontend/boss_map/bframerelation.h"
#include "g2o_frontend/boss_map/bimagesensor.h"
#include "g2o_frontend/boss_map/blasersensor.h"
#include "g2o_frontend/boss_map/bimusensor.h"
#include "g2o_frontend/boss_map/brobot_configuration.h"
#include "g2o_frontend/boss_map/boss_map.h"
#include "g2o_frontend/boss_map/sensing_frame_node.h"
#include "g2o_frontend/boss_map/map_node_processor.h"

#include "two_depthimage_aligner_node.h"

using namespace std;
using namespace g2o;
using namespace Eigen;
using namespace boss;
using namespace boss_map;
using namespace pwn;
using namespace pwn_boss;

std::vector<Serializable*> readConfig(const std::string /* prefix*/, Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile){
  aligner = 0;
  converter = 0;
  Deserializer des;
  des.setFilePath(configFile);
  Serializable* s;
  std::vector<Serializable*> instances;
  cerr << "Reading" << endl;
  while ((s=des.readObject())){
    instances.push_back(s);
    Aligner* al=dynamic_cast<Aligner*>(s);
    if (al) {
      cerr << "got aligner" << endl;
      aligner = al;
    }
    DepthImageConverter* conv=dynamic_cast<DepthImageConverter*>(s);
    if  (conv) {      
      cerr << "got converter" << endl;
      converter = conv;
    }
  }
  if (aligner) {
    cerr << "alpp: " << aligner->projector() << endl;
    cerr << "allz: " << aligner->linearizer() << endl;
    if (aligner->linearizer())
      cerr << "lzal: " << aligner->linearizer()->aligner() << endl;
    
  }

  return instances;
}

Aligner* aligner;
DepthImageConverter* converter;

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string configFile;
  string logFile;
  CommandArgs arg;
  int nscale;
  int mscale;
  
  // Optional input parameters.
  arg.param("nscale", nscale, 1, "image scaling for the normal extraction");
  arg.param("mscale", mscale, 1, "image scaling for matching");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("config_file", configFile, "", "file where the configuration will be written", true);
  arg.paramLeftOver("log_file", logFile, "", "synchronized log file", true);
  arg.parseArgs(argc, argv);

  //Aligner* aligner;
  //DepthImageConverter* converter;

  cerr << "processing log file [" << logFile << "] with parameters read from [" << configFile << "]" << endl;

  cerr << "reading the configuration from file [" << configFile << "]" << endl;
  std::vector<Serializable*> alignerInstances =  readConfig(argv[0], aligner, converter, configFile);

  cerr<< "Aligner: " << aligner << endl;
  cerr<< "Converter: " << converter << endl;


  if (logFile==""){
    cerr << "logfile not supplied" << endl;
  }  


  Deserializer des;
  des.setFilePath(logFile);

  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);

  MapManager* manager = new MapManager;
  SensingFrameNodeMaker* sensingFrameMaker = new SensingFrameNodeMaker;
  sensingFrameMaker->init(manager, conf);

  ImuRelationAdder* imuAdder = new ImuRelationAdder(manager, conf);
  OdometryRelationAdder* odomAdder = new OdometryRelationAdder(manager, conf);
  TwoDepthImageAlignerNode* pairwiseAligner = new TwoDepthImageAlignerNode(manager, conf, converter, aligner,  "/kinect/depth_registered/image_raw");
  for (size_t i = 0; i<sensorDatas.size(); i++) {
    // see if you make a sensing frame with all the infos you find
    SensingFrameNode* s = sensingFrameMaker->processData(sensorDatas[i]);
    if (s) {

      // add a unary relation modeling the imu to the sensing frame
      imuAdder->processNode(s);

      // add a binary relation modeling the odometry between two sensing frames
      odomAdder->processNode(s);

      // add a pwn sensing data(if you manage)
      pairwiseAligner->processNode(s);
    }
  }

  cerr << "writing out things" << endl;
  Serializer ser;
  ser.setFilePath("out.log");
  for (size_t i = 0; i< alignerInstances.size(); i++){
    ser.writeObject(*alignerInstances[i]);
  }
  conf->serializeInternals(ser);
  ser.writeObject(*conf);

  ReferenceFrame* lastFrame = 0;
  for (size_t i = 0; i<sensorDatas.size(); i++) {
    if (lastFrame != sensorDatas[i]->robotReferenceFrame()){
      lastFrame = sensorDatas[i]->robotReferenceFrame();
      ser.writeObject(*lastFrame);
    }
    ser.writeObject(*sensorDatas[i]);
  }
  ser.writeObject(*manager);
  for (std::set<MapNode*>::iterator it = manager->nodes().begin(); it!=manager->nodes().end(); it++)
    ser.writeObject(**it);
  cerr << "writing out " << manager->relations().size() << " relations" << endl;
  for (std::set<MapNodeRelation*>::iterator it = manager->relations().begin(); it!=manager->relations().end(); it++)
    ser.writeObject(**it);

  
  // write the aligner stuff

  // write the robot configuration

  // write the sensor data

  // write the manager

  // write the objects in the manager


  
  // // write back the log
  // Serializer ser;
  // ser.setFilePath("sensing_frame_test.log");

  // conf->serializeInternals(ser);
  // ser.writeObject(*conf);

  // previousReferenceFrame = 0;
  // for (size_t i = 0; i< sensorDatas.size(); i++){
  //   BaseSensorData* data = sensorDatas[i];
  //   if (previousReferenceFrame!=data->robotReferenceFrame()){
  //     ser.writeObject(*data->robotReferenceFrame());
  //   }
  //   ser.writeObject(*data);
  //   previousReferenceFrame = data->robotReferenceFrame();
  // }
  
  // for(size_t i=0; i<newObjects.size(); i++){
  //   ser.writeObject(*newObjects[i]);
  // }

}


   
