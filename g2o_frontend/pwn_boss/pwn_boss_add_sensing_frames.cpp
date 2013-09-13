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
    _mapManager = manager_;
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
      //cerr << "New Frame created" << _currentSensingFrame << endl;
    }
    //cerr << "Payload added" << data->className() << endl;
    _currentSensingFrame->sensorDatas().push_back(data);
    _previousData = data;
    return returned;
  }
};


struct MapNodeProcessor{
  MapNodeProcessor(MapManager* manager_,   RobotConfiguration* config_) {
    _manager = manager_;
    _config = config_;
  }
  MapManager* _manager;
  RobotConfiguration* _config;
  template <class T>
  T* extractRelation(std::vector<MapNode*> nodes){
    if (nodes.size()==0)
      return 0;
    std::set<MapNodeRelation*>& relations = _manager->nodeRelations(nodes[0]);
    for (std::set<MapNodeRelation*>::iterator it=relations.begin(); it!=relations.end(); it++){
      MapNodeRelation* _rel=*it;
      T* rel = dynamic_cast<T*>(_rel);
      if (!rel)
	continue;
      if (rel) {
	for (size_t i=0; rel && i<nodes.size(); i++){
	  if(nodes[i]!=rel->nodes()[i])
	    rel = 0;
	}
	if (rel)
	  return rel;
      }
    }
    return 0;
  }
  virtual void processNode(MapNode* node) = 0;
};


struct ImuRelationAdder : public MapNodeProcessor{
  ImuRelationAdder(MapManager* manager_,   RobotConfiguration* config_): MapNodeProcessor(manager_,config_){}
  virtual void processNode(MapNode* node_){
    SensingFrame* f = dynamic_cast<SensingFrame*>(node_);
    if (!f)
      return;
    for (size_t i = 0; i<f->sensorDatas().size(); i++){
      BaseSensorData* s = f->sensorDatas()[i];
      IMUData* imu = dynamic_cast<IMUData*>(s);
      if (! imu)
	continue;
      
      MapNodeUnaryRelation* rel = new MapNodeUnaryRelation(_manager);
      rel->setGenerator(imu);
      rel->nodes()[0]=f; 
      Eigen::Matrix<double,6,6> info;
      info.setZero();
      info.block<3,3>(3,3)=imu->orientationCovariance().inverse();
      rel->setInformationMatrix(info);
	
      Eigen::Isometry3d iso;
      iso.linear()  = imu->orientation().matrix();
      iso.translation().setZero();
      rel->setTransform(iso);
      cerr << "Imu added (" << f << ")" << endl;
      _manager->addRelation(rel);
    }
  }
};

struct OdometryRelationAdder : public MapNodeProcessor{
  OdometryRelationAdder(MapManager* manager_,   RobotConfiguration* config_): MapNodeProcessor(manager_,config_){
    _previousNode = 0;
  }
  virtual void processNode(MapNode* node_){
    if (_previousNode){
      MapNodeBinaryRelation* rel = new MapNodeBinaryRelation(_manager);
      rel->nodes()[0]=_previousNode;
      rel->nodes()[1]=node_;
      rel->setTransform(_previousNode->transform().inverse()*node_->transform());
      rel->setInformationMatrix(Eigen::Matrix<double,6,6>::Identity());
      _manager->addRelation(rel);
      cerr << "Odom added (" << _previousNode << ", " << node_ << ")" << endl;
      
    }
    _previousNode = node_;
  }
  MapNode* _previousNode;
};

int j = 0;
struct PwnFrameGenerator : public MapNodeProcessor{
  PwnFrameGenerator(MapManager* manager_,   
		    RobotConfiguration* config_,
		    DepthImageConverter* converter_,
		    const std::string& topic_): MapNodeProcessor(manager_,config_){
    _previousSensingFrame = 0;
    _previousFrame = 0;
    _topic = topic_;
    _converter = converter_;
    _scale = 2;
  }

  virtual void processNode(MapNode* node_){
    SensingFrame* sensingFrame = dynamic_cast<SensingFrame*>(node_);
    if (! sensingFrame)
      return;
    
    PinholeImageData* image = dynamic_cast<PinholeImageData*>(sensingFrame->sensorData(_topic));
    if (! image)
      return;
    cerr << "got image"  << endl;
    
    Eigen::Isometry3d _sensorOffset = _config->sensorOffset(image->baseSensor());
    // cerr << "sensorOffset: " << endl;
    // cerr << _sensorOffset.matrix() << endl;

    Eigen::Isometry3f sensorOffset;
    convertScalar(sensorOffset,_sensorOffset);
    sensorOffset.matrix().row(3) << 0,0,0,1;

    Eigen::Matrix3d _cameraMatrix = image->cameraMatrix();
    
    ImageBLOB* blob = image->imageBlob().get();
    
    DepthImage depthImage;
    depthImage.fromCvMat(blob->cvImage());
    int r=depthImage.rows();
    int c=depthImage.cols();
    
    DepthImage scaledImage;
    DepthImage::scale(scaledImage,depthImage,_scale);
    Eigen::Matrix3f cameraMatrix;
    convertScalar(cameraMatrix,_cameraMatrix);
    
    computeScaledParameters(r,c,cameraMatrix,_scale);
    PinholePointProjector* projector=dynamic_cast<PinholePointProjector*>(_converter->_projector);
    cameraMatrix(2,2)=1;
    projector->setCameraMatrix(cameraMatrix);
    pwn::Frame* frame = new pwn::Frame;
    _converter->compute(*frame,scaledImage, sensorOffset);

    MapNodeBinaryRelation* odom=0;

    std::vector<MapNode*> oneNode(1);
    oneNode[0]=sensingFrame;
    MapNodeUnaryRelation* imu = extractRelation<MapNodeUnaryRelation>(oneNode);
    
    if (_previousFrame){
      _aligner->setReferenceSensorOffset(_aligner->currentSensorOffset());
      _aligner->setCurrentSensorOffset(sensorOffset);
      
      _aligner->correspondenceFinder()->setSize(r,c);
      PinholePointProjector* projector=(PinholePointProjector*)(_aligner->projector());
      projector->setCameraMatrix(cameraMatrix);

      /*
      cerr << "correspondenceFinder: "  << r << " " << c << endl; 
      cerr << "sensorOffset" << endl;
      cerr <<_aligner->currentSensorOffset().matrix() << endl;
      cerr <<_aligner->referenceSensorOffset().matrix() << endl;
      cerr << "cameraMatrix" << endl;
      cerr << projector->cameraMatrix() << endl;
      */

      std::vector<MapNode*> twoNodes(2);
      twoNodes[0]=_previousSensingFrame;
      twoNodes[1]=sensingFrame;
      odom = extractRelation<MapNodeBinaryRelation>(twoNodes);
      cerr << "odom:" << odom << " imu:" << imu << endl;

      Eigen::Isometry3f guess;
      _aligner->clearPriors();
      if (odom){
      	Eigen::Isometry3f mean;
      	Eigen::Matrix<float,6,6> info;
      	convertScalar(mean,odom->transform());
      	convertScalar(info,odom->informationMatrix());
      	_aligner->addRelativePrior(mean,info);
	guess = mean;
      } 

      if (imu){
      	Eigen::Isometry3f mean;
      	Eigen::Matrix<float,6,6> info;
      	convertScalar(mean,imu->transform());
      	convertScalar(info,imu->informationMatrix());
      	_aligner->addAbsolutePrior(_globalT,mean,info);
      }
      _aligner->setInitialGuess(guess);
      _aligner->setReferenceFrame(_previousFrame);
      _aligner->setCurrentFrame(frame);
      cerr << "Frames: " << _previousFrame << " " << frame << endl;

      //char buf[1024];
      // sprintf(buf, "frame-%05d.pwn",j);
      // frame->save(buf, 1, true);
      
      // projector->setCameraMatrix(cameraMatrix);
      // projector->setTransform(Eigen::Isometry3f::Identity());
      // Eigen::MatrixXi debugIndices(r,c);
      // DepthImage debugImage(r,c);
      // projector->project(debugIndices, debugImage, frame->points());

      _aligner->align();
      
      // sprintf(buf, "img-dbg-%05d.pgm",j);
      // debugImage.save(buf);
      //sprintf(buf, "img-ref-%05d.pgm",j);
      //_aligner->correspondenceFinder()->referenceDepthImage().save(buf);
      //sprintf(buf, "img-cur-%05d.pgm",j);
      //_aligner->correspondenceFinder()->currentDepthImage().save(buf);

      cerr << "inliers: " << _aligner->inliers() << endl;
      cerr << "chi2: " << _aligner->error() << endl;
      cerr << "chi2/inliers: " << _aligner->error()/_aligner->inliers() << endl;
      cerr << "initialGuess: " << t2v(guess).transpose() << endl;
      cerr << "transform   : " << t2v(_aligner->T()).transpose() << endl;
      if (_aligner->inliers()>100){
 	_globalT = _globalT*_aligner->T();
      }
      Eigen::Matrix3f R = _globalT.linear();
      Eigen::Matrix3f E = R.transpose() * R;
      E.diagonal().array() -= 1;
      _globalT.linear() -= 0.5 * R * E;
      
      cerr << "globalTransform   : " << t2v(_globalT).transpose() << endl;

    } else {
      _aligner->setCurrentSensorOffset(sensorOffset);
      _globalT = Eigen::Isometry3f::Identity();
      if (imu){
      	Eigen::Isometry3f mean;
      	convertScalar(mean,imu->transform());
	_globalT = mean;
      }
    }
    
   
    if (_previousFrame)
      delete _previousFrame;
    
    delete blob;

    _previousSensingFrame = sensingFrame;
    _previousFrame = frame;
    j++;
  }
  DepthImageConverter* _converter;
  Aligner* _aligner;
  SensingFrame* _previousSensingFrame;
  pwn::Frame* _previousFrame;
  std::string _topic;
  BaseSensor* _sensor;
  int _scale;
  Eigen::Isometry3f _globalT;
};


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

  MapManager* manager = new MapManager;
  SensingFrameMaker* sensingFrameMaker = new SensingFrameMaker;
  sensingFrameMaker->init(manager, conf);

  ImuRelationAdder* imuAdder = new ImuRelationAdder(manager, conf);
  OdometryRelationAdder* odomAdder = new OdometryRelationAdder(manager, conf);
  PwnFrameGenerator* frameGenerator = new PwnFrameGenerator(manager, conf, converter,  "/kinect/depth_registered/image_raw");
  frameGenerator->_aligner = aligner;
  for (size_t i = 0; i<sensorDatas.size(); i++) {
    SensingFrame* s = sensingFrameMaker->processData(sensorDatas[i]);
    if (s) {
      imuAdder->processNode(s);
      odomAdder->processNode(s);
      frameGenerator->processNode(s);
    }
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


   
