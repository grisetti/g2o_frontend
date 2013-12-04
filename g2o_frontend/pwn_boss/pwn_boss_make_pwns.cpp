#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/boss_map/bframe.h"
#include "g2o_frontend/boss_map/bframerelation.h"
#include "g2o_frontend/boss_map/bimagesensor.h"
#include "g2o_frontend/boss_map/blasersensor.h"
#include "g2o_frontend/boss_map/bimusensor.h"
#include "g2o_frontend/boss_map/brobot_configuration.h"
#include "pwn_sensor_data.h"

using namespace std;
using namespace g2o;
using namespace Eigen;
using namespace pwn_boss;
using namespace boss;
using namespace pwn;

template <typename T1, typename T2>
void convertScalar(T1& dest, const T2& src){
  for (int i=0; i<src.matrix().cols(); i++)
    for (int j=0; j<src.matrix().rows(); j++)
      dest.matrix()(j,i) = src.matrix()(j,i);

}

void computeScaledParameters (int& rows, int& cols, Eigen::Matrix3f& cameraMatrix, float scale) {
  cameraMatrix.block<2,3>(0,0)*=1./scale;
  rows *=1./scale;
  cols *=1./scale;
}



void readConfig(std::vector<Serializable*>& instances, Aligner*& aligner, DepthImageConverter*& converter, Deserializer& des){
  aligner = 0;
  converter = 0;
  Serializable* s;
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
    if (al && conv){
      aligner= al;
      converter = conv;
      return;
    }
  }
}
Aligner* aligner;
DepthImageConverter* converter;

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string configFile;
  string logFile;
  string outFile;
  CommandArgs arg;
  int scale;
  
  // Optional input parameters.
  arg.param("scale", scale, 4, "image scaling for the normal extraction");
  // Last parameter has to be the working directory.
  arg.paramLeftOver("config_file", configFile, "", "file where the configuration will be written", true);
  arg.paramLeftOver("log_file", logFile, "", "synchronized log file", true);
  arg.paramLeftOver("out_file", outFile, "", "output file", true);
  arg.parseArgs(argc, argv);

  //Aligner* aligner;
  //DepthImageConverter* converter;

  cerr << "processing log file [" << logFile << "] with parameters read from [" << configFile << "]" << endl;

  cerr << "reading the configuration from file [" << configFile << "]" << endl;

  Deserializer des;
  des.setFilePath(configFile);
  std::vector<Serializable*> configInstances;
  readConfig(configInstances, aligner, converter, des);

  cerr<< "Aligner: " << aligner << endl;
  cerr<< "Converter: " << converter << endl;
  if (! aligner || ! converter){
    cerr << "no aligner or converter supplied, aborting" << endl;
    return 0;
  }

  if (logFile==""){
    cerr << "logfile not supplied, aborting" << endl;
    return 0;
  }  

  Deserializer des2;
  des2.setFilePath(logFile);

  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des2);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);


  cerr << "writing algos" << endl;
  Serializer ser;
  ser.setFilePath(outFile);
  for (size_t i = 0; i<configInstances.size(); i++){
    ser.writeObject(*configInstances[i]);
  }
  
  cerr << "writing conf" << endl;
  PWNDepthConverterSensor* convSensor = new PWNDepthConverterSensor(converter);
  BaseSensor* imageSensor = conf->sensor("/kinect/depth_registered/image_raw");
  if (! imageSensor){
    cerr << "sensor with the specified topic is not existing";
    return 0;
  }
  convSensor->setReferenceFrame(imageSensor->frame());
  convSensor->setTopic("/kinect/depth_registered/pwn");
  conf->addSensor(convSensor);
  conf->serializeInternals(ser);
  ser.writeObject(*conf);
  
  cerr << "writing data" << endl;
  PinholePointProjector* proj= (PinholePointProjector*)converter->projector();
  
  ReferenceFrame* lastFrame = 0;
  
  for (size_t i = 0; i<sensorDatas.size(); i++) {
    if (lastFrame != sensorDatas[i]->robotReferenceFrame()){
      lastFrame = sensorDatas[i]->robotReferenceFrame();
      ser.writeObject(*lastFrame);
    }
    if (sensorDatas[i]->topic()=="/kinect/depth_registered/image_raw"){
      PinholeImageData* imageData = dynamic_cast<PinholeImageData*>(sensorDatas[i]);
      if (! imageData)
	return 0;
      ImageBLOB* blob = imageData->imageBlob().get();
      DepthImage image;
      image.fromCvMat(blob->cvImage());
      Eigen::Matrix3f cameraMatrix;
      int r = image.rows();
      int c = image.cols();
      DepthImage scaledImage;
      DepthImage::scale(scaledImage, image, scale);
      convertScalar(cameraMatrix,imageData->cameraMatrix());
      computeScaledParameters(r,c,cameraMatrix,scale);
      proj->setCameraMatrix(cameraMatrix);
      pwn::Frame* f = new pwn::Frame;
      converter->compute(*f,scaledImage,Eigen::Isometry3f::Identity());
      PWNDepthConvertedData* pwnData = new PWNDepthConvertedData(convSensor);
      pwnData->blob().set(f);
      pwnData->_image = imageData;
      pwnData->_scale = scale;
      pwnData->setTimestamp(imageData->timestamp());
      pwnData->setRobotReferenceFrame(imageData->robotReferenceFrame());
      ser.writeObject(*pwnData);
      delete f;
      delete blob;
      cerr << ".";
    }
    ser.writeObject(*sensorDatas[i]);
  }
  
}
