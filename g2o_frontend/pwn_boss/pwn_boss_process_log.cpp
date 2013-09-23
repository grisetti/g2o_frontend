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

#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/boss_logger/bframerelation.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "g2o_frontend/boss_logger/blasersensor.h"
#include "g2o_frontend/boss_logger/bimusensor.h"
#include "g2o_frontend/boss_logger/brobot_configuration.h"

#include "g2o_frontend/boss_map/boss_map.h"


using namespace std;
using namespace g2o;
using namespace Eigen;
using namespace pwn;
using namespace boss;

template <typename T1, typename T2>
void convertScalar(T1& dest, const T2& src){
  for (int i=0; i<src.matrix().cols(); i++)
    for (int j=0; j<src.matrix().rows(); j++)
      dest.matrix()(j,i) = src.matrix()(j,i);
}


std::string filenameExtension(const std::string& s){
    return s.substr(s.find_last_of(".") + 1);
}

std::string baseFilename(const std::string& s){
  return s.substr(0,s.find_last_of("."));
}

void computeScaledParameters (int& rows, int& cols, Eigen::Matrix3f& cameraMatrix, float scale) {
  cameraMatrix.block<2,3>(0,0)*=1./scale;
  rows *=1./scale;
  cols *=1./scale;
}



void readConfig(const std::string /* prefix*/, Aligner*& aligner, DepthImageConverter*& converter, const std::string& configFile){
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

  Serializer ser;
  ser.setFilePath("new.conf");
  for (size_t i=0; i<instances.size(); i++){
    ser.writeObject(*instances[i]);
  }
 
}

struct TSCompare{
  bool operator()(const BaseSensorData* a, const BaseSensorData*b){
    return a->timestamp()<b->timestamp();
  }
};



struct SensingFrameMaker {
  SensingFrameMaker(){
    _mapManager = 0;
    _config = 0;
    _currentSensingFrame = 0;
  }

  void init(MapManager* manager_, RobotConfiguration* config_){
    _mapManager = _mapManager;
    _config = config_;
    _currentSensingFrame = 0;
    _previousData = 0;
  }

  MapManager* _mapManager;
  RobotConfiguration* _config;
  SensingFrame* _currentSensingFrame;
  BaseSensorData* _previousData;

  SensingFrame* processData(BaseSensorData* data){
    SensingFrame * returned = 0;
    if (!_previousData || _previousData->robotReferenceFrame() != data->robotReferenceFrame()){
      // new buddy, push the old one
      returned = _currentSensingFrame;
      _currentSensingFrame = new SensingFrame();
      _currentSensingFrame->setTransform(data->robotReferenceFrame()->transform());
      _mapManager->addNode(_currentSensingFrame);
      cerr << "New Frame created" << _currentSensingFrame << endl;
    }
    cerr << "Payload added" << data->className() << endl;
    _currentSensingFrame->sensorDatas().push_back(data);
    _previousData = data;
    return returned;
  }
};

/*
struct MapNodeProcessor{
  MapManager* _manager;
  RobotConfiguration* _config;
  virtual void processNode(MapNode* node) = 0;
};

struct ImuRelationAdder{
};

struct OdomRelationAdder{
};

struct PWNIncrementalRelationAdder{
};
*/

Aligner* aligner;
DepthImageConverter* converter;
int scale=2;


int numImage = 0;
Eigen::Isometry3f globalT = Eigen::Isometry3f::Identity();

void processSequentialData(SensingFrame* previous, SensingFrame* current, RobotConfiguration* config, const std::string& topic){

  cerr << "got pair of frames, processing: " << previous<< " " << current << endl;
  cerr << "sizes: " << previous->sensorDatas().size() << " " << current->sensorDatas().size() << endl;
  PinholeImageData* pImage = 0;
  PinholeImageData* cImage = 0;
  for (size_t i = 0 ;  i< previous->sensorDatas().size(); i++){
    BaseSensorData* sdata = previous->sensorDatas()[i];
    if (sdata->topic()==topic){
      pImage=dynamic_cast<PinholeImageData*>(sdata);
    }
  }

  for (size_t i = 0 ;  i< current->sensorDatas().size(); i++){
    BaseSensorData* sdata = current->sensorDatas()[i];
    if (sdata->topic()==topic){
      cImage=dynamic_cast<PinholeImageData*>(sdata);
    }
  }

  std::set<MapNodeRelation*>& pRelations = previous->manager()->nodeRelations(previous);
  std::set<MapNodeRelation*>& cRelations = current->manager()->nodeRelations(current);
  MapNodeBinaryRelation* odometry = 0;
  MapNodeUnaryRelation* imu1 = 0, *imu2 = 0;
  for (std::set<MapNodeRelation*>::iterator it=pRelations.begin(); it!=pRelations.end(); it++){
    MapNodeRelation* rel=*it;
    MapNodeUnaryRelation* unary=dynamic_cast<MapNodeUnaryRelation*>(rel);
    MapNodeBinaryRelation* binary=dynamic_cast<MapNodeBinaryRelation*>(rel);
    if (unary)
      imu1 = unary;
    if (binary && binary->nodes()[1]==current)
      odometry = binary;
  }

  for (std::set<MapNodeRelation*>::iterator it=cRelations.begin(); it!=cRelations.end(); it++){
    MapNodeRelation* rel=*it;
    MapNodeUnaryRelation* unary=dynamic_cast<MapNodeUnaryRelation*>(rel);
    MapNodeBinaryRelation* binary=dynamic_cast<MapNodeBinaryRelation*>(rel);
    if (unary)
      imu2 = unary;
    if (binary && binary->nodes()[0]==previous)
      odometry = binary;
  }

  cerr << "Relations size: " << pRelations.size() <<  " " << cRelations.size() <<  endl;
  
  cerr << "img1: " << pImage 
	 << " img2: " << cImage << " imu1: " << imu1 
	 << " imu2: " << imu2 << " odometry: " << odometry << endl;

  if (! (pImage && cImage && imu1 && imu2 && odometry) ){
    cerr << "me haz no data" << endl;
    exit(0);
  }
    
   
  const Eigen::Matrix3d& _cameraMatrix1 = pImage->cameraMatrix();
  const Eigen::Matrix3d& _cameraMatrix2 = cImage->cameraMatrix();
  ImageBLOB* pImageBLOB = pImage->imageBlob().get();
  ImageBLOB* cImageBLOB = cImage->imageBlob().get();

  cerr << "CONVERTING" << endl;
  DepthImage pDepthImage, pScaledImage;
  pDepthImage.fromCvMat(pImageBLOB->cvImage());
  // char iname[1024];
  // sprintf(iname,"img-%05d.pgm", numImage);
  // pDepthImage.save(iname);

  DepthImage cDepthImage, cScaledImage;
  cDepthImage.fromCvMat(cImageBLOB->cvImage());

  DepthImage::scale(pScaledImage, pDepthImage, scale);
  DepthImage::scale(cScaledImage, cDepthImage, scale);
  
  Eigen::Matrix3f pScaledCameraMatrix, cScaledCameraMatrix;
  convertScalar(pScaledCameraMatrix,_cameraMatrix1);
  convertScalar(cScaledCameraMatrix,_cameraMatrix2);
  cScaledCameraMatrix(2,2) = 1.0f;
  pScaledCameraMatrix(2,2) = 1.0f;

  int r1=pDepthImage.rows();
  int c1=pDepthImage.cols();
  int r2=cDepthImage.rows();
  int c2=cDepthImage.cols();
  
  computeScaledParameters(r1,c1,pScaledCameraMatrix, scale);
  computeScaledParameters(r2,c2,cScaledCameraMatrix, scale);

  // cerr << "cameraMatrix: " << endl;
  // cerr << cScaledCameraMatrix << endl;

  cerr << "imsize: " << r1 << " " << c1 << endl;
  aligner->correspondenceFinder()->setSize(r1,c1);

  PinholePointProjector * proj = (PinholePointProjector *)converter->_projector;
  proj->setCameraMatrix(pScaledCameraMatrix);
  //converter->projector()->setCameraMatrix(pScaledCameraMatrix);
  Frame cFrame;
  converter->compute(cFrame,cScaledImage,Eigen::Isometry3f::Identity());
  
  // char fname[1024];
  // sprintf(fname,"frame-%05d.pwn", numImage++);  
  // cFrame.save(fname,1,true);
  Frame pFrame;
  converter->compute(pFrame,pScaledImage,Eigen::Isometry3f::Identity());
  
   
  ReferenceFrame* pSensorFrame = pImage->sensor()->frame();
  ReferenceFrame* cSensorFrame = cImage->sensor()->frame();
  
  boss::StringReferenceFrameMap::const_iterator it=config->frameMap().find(config->baseReferenceFrameId());
  assert(it!=config->frameMap().end());
  ReferenceFrame* baseFrame = it->second;
  
  assert(pSensorFrame->canTransformTo(baseFrame));
  assert(cSensorFrame->canTransformTo(baseFrame));
  
  Eigen::Isometry3d _pSensorOffset = pSensorFrame->transformTo(baseFrame);
  Eigen::Isometry3d _cSensorOffset = cSensorFrame->transformTo(baseFrame);
  
  Eigen::Isometry3f pSensorOffset;
  Eigen::Isometry3f cSensorOffset;
  convertScalar(pSensorOffset, _pSensorOffset);
  convertScalar(cSensorOffset, _cSensorOffset);
  // cerr << "offset: " << endl;
  // cerr << pSensorOffset.matrix() << endl;
  // cerr << cSensorOffset.matrix() << endl;
  //cerr << t2v(pSensorOffset).transpose() << endl;
  cSensorOffset.matrix().row(3) << 0,0,0,1;
  pSensorOffset.matrix().row(3) << 0,0,0,1;
    

  PinholePointProjector* alprojector = (PinholePointProjector*)(aligner->projector());
  alprojector->setCameraMatrix(cScaledCameraMatrix);
  //aligner->setCurrentSensorOffset(cSensorOffset);
  //aligner->setReferenceSensorOffset(pSensorOffset);

  cerr << "ALIGNING" << endl;
  
  aligner->clearPriors();

  aligner->setReferenceFrame(&pFrame);
  aligner->setCurrentFrame(&cFrame);
  

  // determine the initial guess (if present)
  Eigen::Isometry3f guess = Eigen::Isometry3f::Identity();
  guess.matrix().row(3) << 0,0,0,1;
  if (odometry) {
    convertScalar(guess, odometry->transform());
    cerr << "have an odometry guess: " << t2v(guess).transpose() << endl;
    Eigen::Matrix<float,6,6> info;
    convertScalar(info, odometry->informationMatrix());
    aligner->addRelativePrior(guess, info);
  }

  if (imu2) {
    Eigen::Isometry3f iso;
    convertScalar(iso.matrix(), imu2->transform().matrix());
    Eigen::Matrix<float,6,6> info;
    convertScalar(info, imu2->informationMatrix());
    aligner->addAbsolutePrior(globalT, iso, info);
  }
  
  aligner->setInitialGuess(guess);
  aligner->align();

  cerr << "inliers: " << aligner->inliers() << endl;
  cerr << "chi2: " << aligner->error() << endl;
  cerr << "chi2/inliers: " << aligner->error()/aligner->inliers() << endl;
  cerr << "initialGuess: " << t2v(guess).transpose() << endl;
  cerr << "transform   : " << t2v(aligner->T()).transpose() << endl;
  if (aligner->inliers()>100){
    globalT = globalT*aligner->T();
  }
  cerr << "globalTransform   : " << t2v(globalT).transpose() << endl;

  Eigen::Matrix3f R = globalT.linear();
  Eigen::Matrix3f E = R.transpose() * R;
  E.diagonal().array() -= 1;
  globalT.linear() -= 0.5 * R * E;


  delete pImageBLOB;
  delete cImageBLOB;
  
}

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
  readConfig(argv[0], aligner, converter, configFile);

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

  std::vector<Serializable*> newObjects;

  ReferenceFrame* currentReferenceFrame = 0;
  ReferenceFrame* previousReferenceFrame = 0;
  SensingFrame* previousSensingFrame = 0;
  SensingFrame* currentSensingFrame = 0;
  size_t i=0;
  std::vector<MapNodeRelation*> relations;
  MapManager* manager = new MapManager;
  newObjects.push_back(manager);
  int j=0;
  std::vector<SensingFrame*> sensingFrames;
  bool firstImu = false;
  while (i<sensorDatas.size()){
    bool newFrameAdded = 0;
    BaseSensorData* data =sensorDatas[i];
    if(data->robotReferenceFrame() == currentReferenceFrame){
      //cerr << "pushing data to node" << endl;
      cerr << "pushing data to node " << data->className() << endl;
      currentSensingFrame->sensorDatas().push_back(data);
    } else {
      if(currentSensingFrame) {
	//cerr << "adding node to manager (" << j++ << ")" << endl;
	cerr << "# data: " <<  currentSensingFrame->sensorDatas().size() << endl;
      }
      currentSensingFrame = new SensingFrame(manager);
      manager->addNode(currentSensingFrame);
      newFrameAdded=true;
      sensingFrames.push_back(currentSensingFrame);
      newObjects.push_back(currentSensingFrame);
      cerr << "creating new sensing frame (" << currentSensingFrame << ")" << endl;
      cerr << "pushing data to node " << data->className() << endl;
      currentSensingFrame->sensorDatas().push_back(data);
      currentSensingFrame->setTransform(data->robotReferenceFrame()->transform());
    }

    IMUData* imuData = dynamic_cast<IMUData*>(data);
    if (imuData) {
      MapNodeUnaryRelation* rel = new MapNodeUnaryRelation(manager);
      rel->setGenerator(imuData);
      newObjects.push_back(rel);
      rel->nodes()[0]=currentSensingFrame; 
      Eigen::Matrix<double,6,6> info;
      info.setZero();
      info.block<3,3>(3,3)=imuData->orientationCovariance().inverse();
      rel->setInformationMatrix(info);
	
      Eigen::Isometry3d iso;
      iso.linear()  = imuData->orientation().matrix();
      iso.translation().setZero();
      rel->setTransform(iso);
      if (firstImu){
	Eigen::Isometry3f iso;
	convertScalar(iso, rel->transform());
	iso.translation().setZero();
	globalT=iso;
	firstImu=false;
      }
      cerr << "Imu added (" << currentSensingFrame << ")" << endl;
      manager->addRelation(rel);
    }
    if (previousSensingFrame && previousSensingFrame!=currentSensingFrame){
      //cerr << "creating new relation";
      MapNodeBinaryRelation* rel = new MapNodeBinaryRelation(manager);
      newObjects.push_back(rel);
      rel->nodes()[0]=previousSensingFrame;
      rel->nodes()[1]=currentSensingFrame;
      rel->setTransform(previousSensingFrame->transform().inverse()*currentSensingFrame->transform());
      // cerr << "######### Relation Transform: " << previousReferenceFrame << " " << currentReferenceFrame << endl;
      // cerr << rel->transform().matrix() << endl;
      rel->setInformationMatrix(Eigen::Matrix<double,6,6>::Identity());
      //cerr << "adding relation to manager";
      manager->addRelation(rel);
      cerr << "Odom added (" << previousSensingFrame << ", " << currentSensingFrame << ")" << endl;
    }

    previousSensingFrame  = currentSensingFrame;
    previousReferenceFrame = currentReferenceFrame;
    currentReferenceFrame = data->robotReferenceFrame();
    if (newFrameAdded && sensingFrames.size()>3) {
      int previousIndex = sensingFrames.size()-3;
      int currentIndex = sensingFrames.size()-2;
      processSequentialData(sensingFrames[previousIndex], sensingFrames[currentIndex], conf, "/kinect/depth_registered/image_raw");
    }
    i++;
  }


  
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


   
