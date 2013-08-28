#include <iostream>
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "tf/tfMessage.h"
#include <deque>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "image_transport/image_transport.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "g2o_frontend/boss_logger/blasersensor.h"
#include "g2o_frontend/boss_logger/bimusensor.h"

using namespace std;

std::map<std::string, boss::Frame*> tfFrameMap;
std::map<std::string, boss::BaseSensor*> sensorTopicMap;
std::deque<boss::BaseSensorData*> dataQueue;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;

class PinholeImageDataMessageHandler {
public:
  PinholeImageDataMessageHandler(ros::NodeHandle* nh_, std::string topicName_, std::string imageSubtopic, std::string infoSubtopic){
    _nh = nh_;
    _topicName = topicName_;
    _imageTopicName = _topicName + "/" + imageSubtopic;
    _cameraInfoTopicName = _topicName + "/" + infoSubtopic;
    _imageSub = 0;
    _infoSub = 0;
    _sync = 0;
    _sensor = 0;
  }

  virtual ~PinholeImageDataMessageHandler() {
    if (_sync){
      delete _sync;
    }
    if (_infoSub) {
      delete _infoSub;
    }
    if (_imageSub){
      delete _imageSub;
    }
  }

  void subscribe(){
    _imageSub = new message_filters::Subscriber<sensor_msgs::Image>(*_nh, _imageTopicName, 5);
    _infoSub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(*_nh, _cameraInfoTopicName, 5);
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    _sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *_imageSub, *_infoSub);
    _sync->registerCallback(boost::bind(&PinholeImageDataMessageHandler::callback, this, _1, _2));
    cerr << "subscribed topics: [" <<  _imageTopicName << "] and [" << _cameraInfoTopicName << "]" << endl;
  }

  void callback(sensor_msgs::Image::ConstPtr img, sensor_msgs::CameraInfo::ConstPtr info){
    if  (! _sensor) {
      std::map<std::string, boss::Frame*>::iterator it = tfFrameMap.find(info->header.frame_id);
      if (it == tfFrameMap.end()) {
	cerr << "missing transform for frame [" << info->header.frame_id << "], skipping" << endl;
	return;
      }
      _sensor = new boss::PinholeImageSensor;
      _sensor->setTopic(_imageTopicName);
      _sensor->setFrame(it->second);
      Eigen::Matrix3d cmat;
      int i=0;
      for (int r=0; r<3; r++)
	for (int c=0; c<3; c++, i++)
	  cmat(r,c) = info->K[i];
      _sensor->setCameraMatrix(cmat);
      Eigen::VectorXd distParm(info->D.size());
      for (size_t i=0; i<info->D.size(); i++)
	distParm[i]=info->D[i];
      _sensor->setDistortionParameters(distParm);
      _sensor->setDistortionModel(info->distortion_model);
      cerr << "created sensor for topic[" << _sensor->topic()  << "]" << endl;
      sensorTopicMap.insert(make_pair(_sensor->topic(), _sensor));
    }
    // we get the image from ROS

    cv_bridge::CvImagePtr ptr=cv_bridge::toCvCopy(img, img->encoding);
    boss::ImageBLOB* imageBlob = new boss::ImageBLOB();
    imageBlob->cvImage() = ptr->image.clone();
    imageBlob->adjustFormat();
    boss::PinholeImageData* imageData = new boss::PinholeImageData(_sensor);
    imageData->setTimestamp(img->header.stamp.toSec());
    imageData->imageBlob().set(imageBlob);
    dataQueue.push_back(imageData);
  }

protected:
  ros::NodeHandle* _nh;
  message_filters::Subscriber<sensor_msgs::Image>* _imageSub;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* _infoSub;
  message_filters::Synchronizer<MySyncPolicy>* _sync;
  
  std::string _topicName;
  std::string _imageTopicName;
  std::string _cameraInfoTopicName;

  boss::PinholeImageSensor* _sensor;
  sensor_msgs::CameraInfo _cameraInfo;
};


void tfMessageCallback(const tf::tfMessage::ConstPtr& msg){
  for (size_t i=0; i<msg->transforms.size(); i++){
    const geometry_msgs::TransformStamped& t=msg->transforms[i];
    boss::Frame* parentFrame = 0;
    boss::Frame* childFrame = 0;

    std::map<std::string, boss::Frame*>::iterator it = tfFrameMap.find(t.header.frame_id);    
    if (it == tfFrameMap.end()){
      parentFrame = new boss::Frame(t.header.frame_id,Eigen::Isometry3d::Identity(), 0);
      tfFrameMap.insert(std::make_pair(parentFrame->name(), parentFrame));
      cerr << "creating parent frame: " << parentFrame->name() << endl;
    } else {
      parentFrame = it->second;
    }
    it = tfFrameMap.find(t.child_frame_id);
    if (it == tfFrameMap.end()){
      childFrame = new boss::Frame(t.child_frame_id, Eigen::Isometry3d::Identity(), parentFrame);
      tfFrameMap.insert(std::make_pair(childFrame->name(), childFrame));
      cerr << "creating child frame: " << childFrame->name() << endl;
    } else {
      childFrame = it->second;
    }

    Eigen::Isometry3d transform;
    transform.translation().x()=t.transform.translation.x;
    transform.translation().y()=t.transform.translation.y;
    transform.translation().z()=t.transform.translation.z;
    Eigen::Quaterniond rot;
    rot.x()=t.transform.rotation.x;
    rot.y()=t.transform.rotation.y;
    rot.z()=t.transform.rotation.z;
    rot.w()=t.transform.rotation.w;
    transform.linear()=rot.toRotationMatrix();
    childFrame->setTransform(transform);
    if (childFrame->parentFrame()==0) {
      cerr << "reparenting child frame: " << childFrame->name() << " to " << parentFrame->name() << endl;
      childFrame->setParentFrame(parentFrame);
    }
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "bozz");
  ros::NodeHandle nh;
  ros::Subscriber tfSub = nh.subscribe("tf", 1, tfMessageCallback);
  PinholeImageDataMessageHandler kDepthLogger(&nh,"/kinect/depth_registered", "image_raw", "camera_info");
  PinholeImageDataMessageHandler kRGBLogger(&nh,"/kinect/rgb", "image_color", "camera_info");
  kDepthLogger.subscribe();
  kRGBLogger.subscribe();
  while (1){
    ros::spinOnce();
    cerr << "tf: " << tfFrameMap.size() 
	 << ", sensors: "<<sensorTopicMap.size()
	 << ", messages: "<<dataQueue.size() << endl;
    usleep(10000);
  }
}
