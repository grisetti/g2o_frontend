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

using namespace std;
using namespace g2o;
using namespace Eigen;
using namespace pwn;
using namespace boss;



set<string> readDirectory(string dir) {
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  set<string> filenames;
  dp = opendir(dir.c_str());
  if(dp == NULL) {
    return filenames;
  }
  
  while((dirp = readdir(dp))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if(stat(filepath.c_str(), &filestat)) {
      continue;
    }
    if(S_ISDIR(filestat.st_mode)) {
      continue;
    }
    filenames.insert(filepath);
  }

  closedir(dp);

  return filenames;
}


void generateConfig(Aligner*& aligner, DepthImageConverter*& converter, const std::string prefix, const std::string& configFile, const Matrix3f& cameraMatrix){
    PinholePointProjector * convProjector = new PinholePointProjector;
    StatsCalculator* statsCalculator = new StatsCalculator;
    PointInformationMatrixCalculator* pointInformationMatrixCalculator = new PointInformationMatrixCalculator;
    NormalInformationMatrixCalculator* normalInformationMatrixCalculator = new NormalInformationMatrixCalculator;
    converter = new DepthImageConverter (convProjector, statsCalculator, pointInformationMatrixCalculator, normalInformationMatrixCalculator);

    convProjector->setCameraMatrix(cameraMatrix);
    
    
    PinholePointProjector * alProjector = new PinholePointProjector;
    alProjector->setCameraMatrix(cameraMatrix);
    Linearizer * linearizer = new Linearizer;
    CorrespondenceFinder* correspondenceFinder = new CorrespondenceFinder;
    aligner = new Aligner;
    alProjector->setCameraMatrix(cameraMatrix);
    aligner->setProjector(alProjector);
    aligner->setLinearizer(linearizer);
    aligner->setCorrespondenceFinder(correspondenceFinder);
    aligner->setInnerIterations(1);
    aligner->setOuterIterations(10);
    
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


void readConfig(const std::string prefix, Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile){
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
    ser.write(prefix,  *messages[i]->getInstance());
  }
 
}

void alignerRun(Aligner* aligner, DepthImageConverter* converter, std::set<std::string>& pgms){
  int i=0;
  Frame* previousFrame = 0;
  Eigen::Isometry3f currentTransform = Eigen::Isometry3f::Identity();
  for (std::set<string>::iterator it=pgms.begin(); it!=pgms.end(); it++){
    std::string s = *it;
    DepthImage img;
    if (! img.load(s.c_str(), true))
      continue;
    Frame* currentFrame = new Frame;
 
    converter->compute(*currentFrame, img, Eigen::Isometry3f::Identity());
    // char buf[1024];
    // sprintf(buf,"frame-%03d.pwn ",i);
    // currentFrame->save(buf,true);
    cerr << ".";
    if (previousFrame) {
      aligner->setCurrentFrame(currentFrame);
      aligner->setReferenceFrame(previousFrame);
      aligner->setInitialGuess(Eigen::Isometry3f::Identity());
      CorrespondenceFinder * correspondenceFinder = aligner->correspondenceFinder();
      correspondenceFinder->setSize(img.rows(), img.cols());
      aligner->align();
      
      cerr << "o";
      // Eigen::Isometry3f localTransformation = aligner->T();
      // //trajectory = trajectory * localTransformation;
      // //trajectory.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      // cerr << "Local transform : " << endl << localTransformation.matrix() << endl;
      // //cerr << "Global transform: " << endl << trajectory.matrix() << endl;
      // cerr << "Inliers/Minimum number of inliers: " << aligner->inliers() << endl;
      if(aligner->inliers() != 0)
      	cerr << "Error: " << aligner->error() / aligner->inliers() << endl;
      else
      	cerr << "Error: " << std::numeric_limits<float>::max() << endl;
      currentTransform = currentTransform*aligner->T();
    // cerr << "Aligner scaled image size: " << img.rows() << "x" << img.cols() << endl;
      // // If the aligner fails write down the g2o file
      delete previousFrame;
    }
    previousFrame = currentFrame; 
    i++;
  }
  
}

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string sensorType;
  string configFile;
  string dir;
  CommandArgs arg;
  bool gen;
  // Optional input parameters.
  arg.param("sensorType", sensorType, "kinect", "sensor type: xtion640/xtion480/kinect");
  arg.param("gen", gen, false, "read or write the config file");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("config_file", configFile, "", "file where the configuration will be written", true);
  arg.paramLeftOver("dir", dir, ".", "file where the configuration will be written", true);
  arg.parseArgs(argc, argv);

  Aligner* aligner;
  DepthImageConverter* converter;

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
      generateConfig(aligner, converter, argv[0], configFile, cameraMatrix);
    }
  } else {
    cerr << "reading the configuration from file [" << configFile << "]" << endl;
    readConfig(aligner, converter, configFile);
  }

  cerr<< "Aligner: " << aligner << endl;
  cerr<< "Converter: " << converter << endl;
  
  std::set<string> files=readDirectory(dir);
  std::set<string> pgms;
  
  for (std::set<string>::iterator it=files.begin(); it!=files.end(); it++){
    std::string s = *it;
    std::string extension = s.substr(s.find_last_of(".") + 1);
    if (extension == "pgm" || extension == "PGM")
      pgms.insert(s);
  }

  alignerRun(aligner, converter, pgms);
}


   
