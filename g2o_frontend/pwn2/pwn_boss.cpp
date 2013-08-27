#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

using namespace std;
using namespace g2o;
using namespace Eigen;
using namespace pwn;
using namespace boss;


void generateConfig(const std::string prefix, const std::string& configFile, const Matrix3f& cameraMatrix){
    PinholePointProjector * convProjector = new PinholePointProjector;
    StatsCalculator* statsCalculator = new StatsCalculator;
    PointInformationMatrixCalculator* pointInformationMatrixCalculator = new PointInformationMatrixCalculator;
    NormalInformationMatrixCalculator* normalInformationMatrixCalculator = new NormalInformationMatrixCalculator;
    DepthImageConverter* converter = new DepthImageConverter (convProjector, statsCalculator, pointInformationMatrixCalculator, normalInformationMatrixCalculator);

    convProjector->setCameraMatrix(cameraMatrix);
    
    
    PinholePointProjector * alProjector = new PinholePointProjector;
    Linearizer * linearizer = new Linearizer;
    CorrespondenceFinder* correspondenceFinder = new CorrespondenceFinder;
    Aligner* aligner = new Aligner;
    aligner->setProjector(alProjector);
    alProjector->setCameraMatrix(cameraMatrix);
    aligner->setLinearizer(linearizer);
    aligner->setCorrespondenceFinder(correspondenceFinder);
  
    Serializer ser;
    ser.setFilePath(configFile);
    ser.writeObject(*convProjector);
    ser.writeObject(*statsCalculator);
    ser.writeObject(*pointInformationMatrixCalculator);
    ser.writeObject(*normalInformationMatrixCalculator);
    ser.writeObject(*converter);
    ser.writeObject(*alProjector);
    ser.writeObject(*linearizer);
    ser.writeObject(*correspondenceFinder);
    ser.writeObject(*aligner);
}


void readConfig(const std::string prefix, Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile){
  aligner = 0;
  converter = 0;
  Deserializer des;
  des.setFilePath(configFile);
  Serializable* s;
  std::vector<Serializable*> instances;
  cerr << "Reading" << endl;
  while ((s=des.readObject())){
    cerr << "Got " << s->className() << endl;
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

  Serializer ser;
  ser.setFilePath("new.conf");
  for (int i=0; i<instances.size(); i++){
    cerr << i << endl;
    ser.writeObject(*instances[i]);
  }
 
}

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string sensorType;
  string configFile;
  CommandArgs arg;
  bool gen;
  // Optional input parameters.
  arg.param("sensorType", sensorType, "kinect", "sensor type: xtion640/xtion480/kinect");
  arg.param("gen", gen, false, "read or write the config file");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("config_file", configFile, "", "file where the configuration will be written", true);
  arg.parseArgs(argc, argv);

    Matrix3f cameraMatrix;
    Eigen::Isometry3f sensorOffset = Isometry3f::Identity();

    // Set sensor offset
    sensorOffset.translation() = Vector3f(0.0f, 0.0f, 0.0f);
    Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
    sensorOffset.linear() = quaternion.toRotationMatrix();
    sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    cameraMatrix.setIdentity();

    if (sensorType=="xtion640") {
      cameraMatrix << 
	570.342, 0,       320,
	0,       570.342, 240,
	0.0f, 0.0f, 1.0f;  
    }  else if (sensorType=="xtion320") {
      cameraMatrix << 
	570.342, 0,       320,
	0,       570.342, 240,
	0.0f, 0.0f, 1.0f;  
      cameraMatrix.block<2,3>(0,0)*=0.5;
    } else if (sensorType=="kinect") {
      cameraMatrix << 
	525.0f, 0.0f, 319.5f,
	0.0f, 525.0f, 239.5f,
	0.0f, 0.0f, 1.0f;  
    } else {
      cerr << "unknown sensor type: [" << sensorType << "], aborting (you need to specify either xtion or kinect)" << endl;
      return 0;
    }
 
  if (gen) {
    if (configFile.length()) {
      cerr << "writing the configuration in file [" << configFile << "]" << endl;
      generateConfig(argv[0], configFile, cameraMatrix);
    }
  } else {
    cerr << "reading the configuration from file [" << configFile << "]" << endl;
    Aligner* aligner;
    DepthImageConverter* converter;
    readConfig(argv[0], aligner, converter, configFile);
    cerr<< "Aligner: " << aligner << endl;
    cerr<< "Converter: " << converter << endl;

    // retrieve the con
  }

}


   
