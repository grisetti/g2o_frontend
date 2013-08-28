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


std::string filenameExtension(const std::string& s){
    return s.substr(s.find_last_of(".") + 1);
}

std::string baseFilename(const std::string& s){
  return s.substr(0,s.find_last_of("."));
}

void computeScaledParameters (int& rows, int& cols, Eigen::MatrixXf& cameraMatrix, float scale) {
  cameraMatrix.block<2,3>(0,0)*=scale;
  rows *=scale;
  cols *=scale;
}



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


void generateConfig(Aligner*& aligner, DepthImageConverter*& converter, const std::string prefix, const std::string& configFile, const Matrix3f& cameraMatrix, int nscale, int mscale){
  int rows=640;
  int cols=480;
  
  int nRows = rows;
  int nCols = cols;
  MatrixXf nCameraMatrix = cameraMatrix;
  computeScaledParameters(nRows, nCols, nCameraMatrix, 1./nscale);


  PinholePointProjector * convProjector = new PinholePointProjector;
  StatsCalculator* statsCalculator = new StatsCalculator;
  PointInformationMatrixCalculator* pointInformationMatrixCalculator = new PointInformationMatrixCalculator;
  NormalInformationMatrixCalculator* normalInformationMatrixCalculator = new NormalInformationMatrixCalculator;
  converter = new DepthImageConverter (convProjector, statsCalculator, pointInformationMatrixCalculator, normalInformationMatrixCalculator);
  convProjector->setCameraMatrix(nCameraMatrix);
  
    


  MatrixXf mCameraMatrix = cameraMatrix;
  int mRows = rows;
  int mCols = cols;
  computeScaledParameters(mRows, mCols, mCameraMatrix, 1./mscale);
    
  PinholePointProjector * alProjector = new PinholePointProjector;
  alProjector->setCameraMatrix(cameraMatrix);
  Linearizer * linearizer = new Linearizer;
  CorrespondenceFinder* correspondenceFinder = new CorrespondenceFinder;
  correspondenceFinder->setSize(mRows, mCols);
  aligner = new Aligner;
  alProjector->setCameraMatrix(mCameraMatrix);
  aligner->setProjector(alProjector);
  aligner->setLinearizer(linearizer);
  aligner->setCorrespondenceFinder(correspondenceFinder);
  aligner->setInnerIterations(1);
  aligner->setOuterIterations(10);
  
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

void alignerRun(Aligner* aligner, DepthImageConverter* converter, const std::string& filename,  const std::set<std::string>& pgms, int scale){
  int i=0;
  Frame* previousFrame = 0;
  Eigen::Isometry3f globalT = Eigen::Isometry3f::Identity();
  DepthImage img;
  DepthImage scaledImage;
  ofstream os(filename.c_str());
  for (std::set<string>::const_iterator it=pgms.begin(); it!=pgms.end(); it++){
    std::string s = *it;
    if (! img.load(s.c_str(), true))
      continue;
    DepthImage::scale(scaledImage, img, scale);

    Frame* currentFrame = new Frame;
    converter->compute(*currentFrame, scaledImage, Eigen::Isometry3f::Identity());
    // char buf[1024];
    // sprintf(buf,"frame-%03d.pwn ",i);
    // currentFrame->save(buf,true);
    cerr << ".";
    if (previousFrame) {
      aligner->setCurrentFrame(currentFrame);
      aligner->setReferenceFrame(previousFrame);
      aligner->setInitialGuess(Eigen::Isometry3f::Identity());
      aligner->align();
      
      cerr << "o";
      globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      globalT = globalT*aligner->T();
      
      Eigen::Matrix3f R = globalT.linear();
      Eigen::Matrix3f E = R.transpose() * R;
      E.diagonal().array() -= 1;
      globalT.linear() -= 0.5 * R * E;

      os << "FRAME " << i << " " << s << " " << aligner->inliers() << " " << aligner->error() << " ";
      if (aligner->inliers())
	os << aligner->error()/aligner->inliers();
      else
	os << "nan";
      os <<" " << t2v(aligner->T()).transpose() << " " << t2v(globalT).transpose() << endl;

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
  int nscale;
  int mscale;
  
  // Optional input parameters.
  arg.param("sensorType", sensorType, "kinect", "sensor type: xtion640/xtion480/kinect");
  arg.param("gen", gen, false, "read or write the config file");
  arg.param("nscale", nscale, 1, "image scaling for the normal extraction");
  arg.param("mscale", mscale, 1, "image scaling for matching");

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
      generateConfig(aligner, converter, argv[0], configFile, cameraMatrix, nscale, mscale);
    }
  } else {
    cerr << "reading the configuration from file [" << configFile << "]" << endl;
    readConfig(argv[0], aligner, converter, configFile);
  }

  cerr<< "Aligner: " << aligner << endl;
  cerr<< "Converter: " << converter << endl;
  
  std::set<string> files=readDirectory(dir);
  std::set<string> pgms;
  
  for (std::set<string>::iterator it=files.begin(); it!=files.end(); it++){
    std::string extension = filenameExtension(*it);
    if (extension == "pgm" || extension == "PGM")
      pgms.insert(*it);
  }
  files.clear();
  std::string outFilename=baseFilename(configFile)+".txt";
  alignerRun(aligner, converter, outFilename, pgms, mscale);
}


   
