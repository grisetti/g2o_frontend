#ifndef _ROS_BOSS_MESSAGE_CONTEXT_H_
#define _ROS_BOSS_MESSAGE_CONTEXT_H_

#include "ros/ros.h"
#include "g2o_frontend/boss_logger/bsensor.h"
#include "g2o_frontend/boss_logger/brobot_configuration.h"
#include "tf/transform_listener.h"
using namespace std;

class RosMessageHandler;
class RosTransformMessageHandler;
class RosMessageContext : public boss::RobotConfiguration {
public:
  RosMessageContext(ros::NodeHandle* nh_);

  virtual ~RosMessageContext();

  inline ros::NodeHandle* nodeHandle() {return _nh;}
  boss::StringSensorMap& sensorMap() {return _sensorMap;}
  boss::StringReferenceFrameMap&  frameMap() {return _frameMap;}
  boss::SerializableQueue& messageQueue() {return _messageQueue;}
  bool addHandler(const std::string& type, const std::string& topic);
  RosMessageHandler* handler(const std::string topic);
  bool configReady() const;
  void init();

  bool getOdomPose(Eigen::Isometry3d& t, double time);
  inline void setOdomReferenceFrameId(const std::string odomReferenceFrameId_) {_odomReferenceFrameId = odomReferenceFrameId_;}
  inline const std::string& odomReferenceFrameId() const {return _odomReferenceFrameId;}
  
  
protected:
  ros::NodeHandle* _nh;
  tf::TransformListener* _tfListener;
  RosTransformMessageHandler* _transformHandler;
  boss::SerializableQueue  _messageQueue;
  std::string _odomReferenceFrameId;
  std::map<std::string, RosMessageHandler*> _handlers;
  
};

#endif
