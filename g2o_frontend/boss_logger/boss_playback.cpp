#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"
#include "bimagesensor.h"
#include "blasersensor.h"
#include "bimusensor.h"

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>





using namespace boss;
using namespace std;

Frame f;
PinholeImageSensor i;
LaserSensor s;
IMUSensor imu;

StringSensorMap sensors;
StringFrameMap  frames;
std::vector<boss::Serializable*> objects;
std::vector<BaseSensorData*> sensorDatas;

struct TSCompare{
  bool operator()(const BaseSensorData* a, const BaseSensorData*b){
    return a->timestamp()<b->timestamp();
  }
};

class MySimpleVisualizer{
public:
  MySimpleVisualizer(const std::string& windowName_) {
    _windowName = windowName_;
    cv::namedWindow(_windowName.c_str(),1);
  }
  virtual ~MySimpleVisualizer(){}
  virtual bool show(BaseSensorData* sdata) {return false;}
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
    cerr << "prentento lo plop" <<endl;
    ImageBLOB* imblob = image->imageBlob().get();
    cerr << "presp lo plop: " << imblob << endl;
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
    // cerr << "minRange" << laser->minRange() << endl;
    // cerr << "maxRange" << laser->maxRange() << endl;
    // cerr << "theta" << theta << endl;
    // cerr << "res" << res << endl;
    // cerr << "size" << laser->ranges().size() << endl;
    Eigen::Isometry3d worldToLaser = laser->robotFrame()->transform().inverse();
    Eigen::Isometry3d rotateThing = laser->robotFrame()->transform();
    rotateThing.translation() << 0,0,0;
    Eigen::Isometry3d doStuff = rotateThing*worldToLaser;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;
    for (int i=0; i<laser->ranges().size(); i++){
      float d = laser->ranges()[i];
      if (d>laser->minRange() && d<laser->maxRange()){
	float x =  d*cos(theta);
	float y =  d*sin(theta);
	points.push_back(Eigen::Vector3d(x,y,0));
      }
      theta += res;
    }
    for (int i=_trajectory->size()-1; i>0; i--){
      Eigen::Vector3d p=doStuff*_trajectory->at(i);
      float x =  p.x()*pixelsPerMeter+xcenter;
      float y = -p.y()*pixelsPerMeter+ycenter;
	if (x>0 && x<laserImage.cols && y>0 && y < laserImage.rows) {
	  laserImage.at<unsigned char>((int) y, (int) x) = 127;
	}
    }
    for (int i=0; i<points.size(); i++){
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


std::map<BaseSensor*, MySimpleVisualizer*> visualizers;

int main(int argc, char** argv) {
  Deserializer des;
  des.setFilePath("./test.log");
  Serializable *o;

  while( (o=des.readObject()) ){
    cerr << ".";
    BaseSensor* sensor= dynamic_cast<BaseSensor*>(o);
    if (sensor) {
      sensors.insert(make_pair(sensor->topic(), sensor));
    }

    Frame* frame=dynamic_cast<Frame*>(o);
    if (frame && frame->name()!=""){
      frames.insert(make_pair(frame->name(), frame));
    }
    
    BaseSensorData* sensorData=dynamic_cast<BaseSensorData*>(o);
    if (sensorData){
      sensorDatas.push_back(sensorData);
      // LaserData* laserData = dynamic_cast<LaserData*>(o);
      // if (laserData) {
      // 	delete laserData;
      // 	o = 0;
      // 	sensorData = 0;
      // }      
      // IMUData* imuData = dynamic_cast<IMUData*>(o);
      // if (imuData){
      // 	delete imuData; 
      // 	o = 0;
      // 	sensorData = 0;
      // }
      // if (sensorData)
      // 	sensorDatas.push_back(sensorData);
    }
    objects.push_back(o);
  }
  cerr << "read: " << objects.size() << " objects"  << endl;
  cerr << "# frames: " << frames.size() << endl;
  cerr << "# sensors: " << sensors.size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;
  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > trajectory;
  //instantiate the visualizers;
  for(StringSensorMap::iterator it=sensors.begin(); it!=sensors.end(); it++){
    BaseSensor* sensor = it->second;
    PinholeImageSensor* pinholeSensor = dynamic_cast<PinholeImageSensor*>(sensor);
    if (pinholeSensor)
      visualizers.insert(make_pair(pinholeSensor, new MySimpleImageVisualizer(pinholeSensor->topic())));
    LaserSensor* laserSensor = dynamic_cast<LaserSensor*>(sensor);
    if (laserSensor)
      visualizers.insert(make_pair(laserSensor, new MySimpleLaserVisualizer(laserSensor->topic(), &trajectory)));
  }


  int i = 0;
  while (1) {
    char c;
    c= cv::waitKey(0);
    if (c == 27)
      return 0;
    if (c == 'n' && i<sensorDatas.size()-1){
      i++;
      BaseSensorData* data =sensorDatas[i];
      std::map<BaseSensor*,MySimpleVisualizer*>::iterator visIt = visualizers.find(data->baseSensor());
      Eigen::Isometry3d T=data->robotFrame()->transform();
      if (visIt!=visualizers.end()) {
	visIt->second->show(data);
      }
      trajectory.push_back(T.translation());
    }
    if (c == 'p' && i>1){
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
