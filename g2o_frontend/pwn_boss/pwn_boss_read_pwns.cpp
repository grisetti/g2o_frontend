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
    cerr << s->className() << endl;
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
    if (aligner && converter){
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
  string logFile;
  CommandArgs arg;
  int scale;
  
  // Optional input parameters.
  arg.param("scale", scale, 4, "image scaling for the normal extraction");
  // Last parameter has to be the working directory.
  arg.paramLeftOver("log_file", logFile, "", "synchronized log file", true);
  arg.parseArgs(argc, argv);

  //Aligner* aligner;
  //DepthImageConverter* converter;

  cerr << "processing log file [" << logFile << "]" << endl;
  
  Deserializer des;
  des.setFilePath(logFile);
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
  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);
  
  for (size_t i = 0; i<sensorDatas.size(); i++) {
    PWNDepthConvertedData* pwnData = dynamic_cast<PWNDepthConvertedData*>(sensorDatas[i]);
    printf("%10.10f\n",sensorDatas[i]->timestamp());
    if (pwnData) {
      //pwn::Frame* f = pwnData->blob().get();
      cerr << ".";
      //delete f;
    } else
      cerr << "o" << endl;
    
  }
  cerr << "done" << endl;
  
}
