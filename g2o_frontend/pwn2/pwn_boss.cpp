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
    ser.write(prefix, *convProjector);
    ser.write(prefix, *statsCalculator);
    ser.write(prefix, *pointInformationMatrixCalculator);
    ser.write(prefix, *normalInformationMatrixCalculator);
    ser.write(prefix, *converter);
    ser.write(prefix, *alProjector);
    ser.write(prefix, *linearizer);
    ser.write(prefix, *correspondenceFinder);
    ser.write(prefix, *aligner);
}


void readConfig(Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile){
  aligner = 0;
  converter = 0;
  Deserializer des;
  des.setFilePath(configFile);
  Message* m;
  std::vector<Message*> messages;
  while ((m=des.readMessage())){
    messages.push_back(m);
    Serializable* s = m->getInstance();
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
  for (int i=0; i<messages.size(); i++){
    cerr << i << endl;
    ser.write("boh",  *messages[i]->getInstance());
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
  arg.param("gen", gen, "false", "read or write the config file");

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
    readConfig(aligner, converter, configFile);
    cerr<< "Aligner: " << aligner << endl;
    cerr<< "Converter: " << converter << endl;

    // retrieve the con
  }

}


   
