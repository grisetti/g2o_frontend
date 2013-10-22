#include "ros_message_context.h"
#include "ros_transform_message_handler.h"
#include "ros_pinholeimagedata_message_handler.h"
#include "ros_laser_message_handler.h"
#include "ros_imu_data_message_handler.h"

RosMessageContext::RosMessageContext(ros::NodeHandle* nh_) {
  _nh = nh_;
  _transformHandler = new RosTransformMessageHandler(this);
  _tfListener = new tf::TransformListener(*_nh, ros::Duration(30.0));
  _odomReferenceFrameId = "/odom";
  _baseReferenceFrameId = "/base_link";
}

RosMessageContext::RosMessageContext(ros::NodeHandle* nh_, boss_logger::RobotConfiguration* robotConfiguration) {
  _nh = nh_;
  _transformHandler = new RosTransformMessageHandler(this);
  _tfListener = new tf::TransformListener(*_nh, ros::Duration(30.0));
  _odomReferenceFrameId = "/odom";
  _baseReferenceFrameId = "/base_link";
  this->_isReady = robotConfiguration->isReady();
  const StringSensorMap& sensorMap = robotConfiguration->sensorMap(); 
  this->_sensorMap = sensorMap;
  const StringReferenceFrameMap& frameMap = robotConfiguration->frameMap();
  this->_frameMap = frameMap;
  this->_baseReferenceFrameId = robotConfiguration->baseReferenceFrameId();
  const std::string& name = robotConfiguration->name();
  this->_name = name;
}

RosMessageContext::~RosMessageContext(){
  delete _tfListener;
}

bool RosMessageContext::getOdomPose(Eigen::Isometry3d& _trans, double time){
  bool transformFound = true;
  _tfListener->waitForTransform(_odomReferenceFrameId, _baseReferenceFrameId,
				ros::Time(time), ros::Duration(1.0));
  try{
    tf::StampedTransform t;
    _tfListener->lookupTransform(_odomReferenceFrameId, _baseReferenceFrameId,
				 ros::Time(time), t);
    Eigen::Isometry3d transform;
    transform.translation().x()=t.getOrigin().x();
    transform.translation().y()=t.getOrigin().y();
    transform.translation().z()=t.getOrigin().z();
    Eigen::Quaterniond rot;
    rot.x()=t.getRotation().x();
    rot.y()=t.getRotation().y();
    rot.z()=t.getRotation().z();
    rot.w()=t.getRotation().w();
    transform.linear()=rot.toRotationMatrix();
    _trans = transform;
    transformFound = true;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    transformFound = false;
  }
  return transformFound;
}


RosMessageHandler* RosMessageContext::handler(const std::string topic) {
  std::map<std::string, RosMessageHandler*>::iterator it = _handlers.find(topic);
  if (it==_handlers.end())
    return 0;
  return it->second;
}

bool RosMessageContext::addHandler(const std::string& type, const std::string& topic) {
  if (type == "laser") {
    RosLaserDataMessageHandler* rosLaserMessageHandler = new RosLaserDataMessageHandler(this, topic);
    StringSensorMap::iterator it = _sensorMap.find(topic);
    if (it == _sensorMap.end()) {
      cerr << "WARNING: laser sensor not found while adding laser message handler" << endl;
    }
    else {
      rosLaserMessageHandler->setSensor(it->second);
    }
    cerr <<"added handler for topic" << topic << endl;
    _handlers.insert(make_pair(topic, rosLaserMessageHandler));
    return true;
  }
  if (type == "image") {
    RosPinholeImageDataMessageHandler* rosPinholeImageMessageHandler = new RosPinholeImageDataMessageHandler(this, topic);
    StringSensorMap::iterator it = _sensorMap.find(topic);
    if (it == _sensorMap.end()) {
      cerr << "WARNING: camera sensor not found while adding camera message handler" << endl;
    }
    else {
      rosPinholeImageMessageHandler->setSensor(it->second);
    }
    cerr << "added handler for topic" << topic << endl;
    _handlers.insert(make_pair(topic, rosPinholeImageMessageHandler));
    return true;
  }
  if (type == "imu") {
    RosIMUDataMessageHandler* rosIMUMessageHandler = new RosIMUDataMessageHandler(this, topic);
    StringSensorMap::iterator it = _sensorMap.find(topic);
    if (it == _sensorMap.end()) {
      cerr << "WARNING: IMU sensor not found while adding imu message handler" << endl;
    }
    else {
      rosIMUMessageHandler->setSensor(it->second);
    }
    cerr << "added handler for topic" << topic << endl;
    _handlers.insert(make_pair(topic, rosIMUMessageHandler));
    return true;
  }

  cerr << "WARNING: unknown handler type [" << type << "]" <<  endl;
  return false;
}

bool RosMessageContext::configReady() const{
  bool allReady=true;
  for (std::map<std::string, RosMessageHandler*>::const_iterator it = _handlers.begin(); it!=_handlers.end(); it++){
    const RosMessageHandler* handler=it->second;
    allReady &= handler->configReady();
  }
  return allReady;
}

void RosMessageContext::initSubscribers(){
  _transformHandler->subscribe();
  for (std::map<std::string, RosMessageHandler*>::iterator it = _handlers.begin(); it!=_handlers.end(); it++){
    RosMessageHandler* handler=it->second;
    handler->subscribe();
  }
}

void RosMessageContext::initPublishers(){
  _transformHandler->advertise();
  for (std::map<std::string, RosMessageHandler*>::iterator it = _handlers.begin(); it!=_handlers.end(); it++){
    RosMessageHandler* handler=it->second;
    handler->advertise();
  }
}
