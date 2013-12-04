#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_map/bframe.h"
#include "g2o_frontend/boss_map/bframerelation.h"
#include "g2o_frontend/boss_map/bimagesensor.h"
#include "g2o_frontend/boss_map/blasersensor.h"
#include "g2o_frontend/boss_map/bimusensor.h"
#include "g2o_frontend/boss_map/brobot_configuration.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// just to make the linker happy
#include "g2o_frontend/pwn_boss/pwn_sensor_data.h"

pwn_boss::PWNSensorData data;

using namespace boss_map;
using namespace boss;
using namespace std;

ReferenceFrame f;
PinholeImageSensor i;
LaserSensor s;
IMUSensor imu;

StringSensorMap sensors;
StringReferenceFrameMap  frames;
std::vector<boss::Serializable*> objects;
std::vector<BaseSensorData*> sensorDatas;

class MySimpleVisualizer{
public:
  MySimpleVisualizer(const std::string& windowName_) {
    _windowName = windowName_;
    cv::namedWindow(_windowName.c_str(),1);
  }
  virtual ~MySimpleVisualizer(){}
  virtual bool show(BaseSensorData* ) {return false;}
protected:
  std::string _windowName;
};

class MySimpleImageVisualizer: public MySimpleVisualizer {
public:
  MySimpleImageVisualizer(const std::string& windowName_): MySimpleVisualizer(windowName_){}
  virtual bool show(BaseSensorData* sdata) {
    PinholeImageData* image = dynamic_cast<PinholeImageData*>(sdata);
    if (!image)
      return false;
    ImageBLOB* imblob = image->imageBlob().get();
    imshow(_windowName.c_str(), imblob->cvImage());
    delete imblob;
    return true;
  };
};

class MySimpleLaserVisualizer: public MySimpleVisualizer {
public:
  MySimpleLaserVisualizer(const std::string& windowName_,
			  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >* trajectory_): MySimpleVisualizer(windowName_){
    _trajectory = trajectory_;
    laserImage=cv::Mat(1000,1000,CV_8UC1);
  }

  virtual bool show(BaseSensorData* sdata) {
    LaserData* laser = dynamic_cast<LaserData*>(sdata);
    if (!laser)
      return false;
    memset(laserImage.data,0,laserImage.rows*laserImage.cols);
    float theta = laser->minAngle();
    float res = laser->angularResolution();
    float metersPerPixel = 0.05;
    float pixelsPerMeter = 1./metersPerPixel;
    int xcenter = laserImage.rows/2;
    int ycenter = laserImage.rows/2;
    Eigen::Isometry3d worldToLaser = laser->robotReferenceFrame()->transform().inverse();
    Eigen::Isometry3d rotateThing = laser->robotReferenceFrame()->transform();
    rotateThing.translation() << 0,0,0;
    Eigen::Isometry3d doStuff = rotateThing*worldToLaser;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;
    for (size_t i=0; i<laser->ranges().size(); i++){
      float d = laser->ranges()[i];
      if (d>laser->minRange() && d<laser->maxRange()){
	float x =  d*cos(theta);
	float y =  d*sin(theta);
	points.push_back(Eigen::Vector3d(x,y,0));
      }
      theta += res;
    }

    for (size_t i=0; i<_trajectory->size(); i++){
      Eigen::Vector3d p=doStuff*_trajectory->at(i);
      float x =  p.x()*pixelsPerMeter+xcenter;
      float y = -p.y()*pixelsPerMeter+ycenter;
    	if (x>0 && x<laserImage.cols && y>0 && y < laserImage.rows) {
    	  laserImage.at<unsigned char>((int) y, (int) x) = 127;
    	}
    }
    for (size_t i=0; i<points.size(); i++){
      Eigen::Vector3d p=rotateThing*points.at(i);
      float x =  p.x()*pixelsPerMeter+xcenter;
      float y = -p.y()*pixelsPerMeter+ycenter;
    	if (x>0 && x<laserImage.cols && y>0 && y < laserImage.rows) {
    	  laserImage.at<unsigned char>((int) y, (int) x) = 255;
    	}
    }

      
    //cerr << endl;
    imshow(_windowName.c_str(), laserImage);

    return true;
  };
  cv::Mat laserImage;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >* _trajectory;
};




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


std::map<BaseSensor*, MySimpleVisualizer*> visualizers;

int main(int argc, char** argv) {
  Deserializer des;
  
  if (argc<2){
    printBanner();
    return 0;
  }

    
  des.setFilePath(argv[1]);

  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > trajectory;
  //instantiate the visualizers;
  for(StringSensorMap::const_iterator it=conf->sensorMap().begin(); it!=conf->sensorMap().end(); it++){
    BaseSensor* sensor = it->second;
    PinholeImageSensor* pinholeSensor = dynamic_cast<PinholeImageSensor*>(sensor);
    if (pinholeSensor)
      visualizers.insert(make_pair(pinholeSensor, new MySimpleImageVisualizer(pinholeSensor->topic())));
    LaserSensor* laserSensor = dynamic_cast<LaserSensor*>(sensor);
    if (laserSensor)
      visualizers.insert(make_pair(laserSensor, new MySimpleLaserVisualizer(laserSensor->topic(), &trajectory)));
  }

  cerr << "created visaualizers" << endl;

  ReferenceFrame* currentReferenceFrame = 0;
  int i=0;
  while (1) {
    char c;
    c= cv::waitKey(0);
    BaseSensorData* data =sensorDatas[i];
    currentReferenceFrame = data->robotReferenceFrame();
    if (c == 27)
      return 0;
    if (c == 'n'){
      while (sensorDatas[i]->robotReferenceFrame()==currentReferenceFrame && i<sensorDatas.size() - 1) {
	i++;
	BaseSensorData* data =sensorDatas[i];
	std::map<BaseSensor*,MySimpleVisualizer*>::iterator visIt = visualizers.find(data->baseSensor());
	//cerr << "data->robotReferenceFrame(): " << data->robotReferenceFrame() << endl;
	Eigen::Isometry3d T=data->robotReferenceFrame()->transform();
	if (visIt!=visualizers.end()) {
	  visIt->second->show(data);
	}
	trajectory.push_back(T.translation());
      }
    }
    if (c == 'p'){
      while (sensorDatas[i]->robotReferenceFrame()==currentReferenceFrame && i>1) {
	i--;
	BaseSensorData* data =sensorDatas[i];
	std::map<BaseSensor*,MySimpleVisualizer*>::iterator visIt = visualizers.find(data->baseSensor());
	if (visIt!=visualizers.end()) {
	  visIt->second->show(data);
	}
	trajectory.pop_back();
      }
    }
  }
}
