#ifndef _ROS_BOSS_MESSAGE_CONTEXT_H_
#define _ROS_BOSS_MESSAGE_CONTEXT_H_

#include "ros/ros.h"
#include "g2o_frontend/boss_logger/bsensor.h"
#include "tf/transform_listener.h"
using namespace std;

class RosMessageContext {
public:
  RosMessageContext(ros::NodeHandle* nh_);

  virtual ~RosMessageContext();

  inline ros::NodeHandle* nodeHandle() {return _nh;}
  boss::StringSensorMap& sensorMap() {return _sensorMap;}
  boss::StringFrameMap&  frameMap() {return _frameMap;}
  boss::SerializableQueue& messageQueue() {return _messageQueue;}
  bool getOdomPose(Eigen::Isometry3d& t, double time);

protected:
  ros::NodeHandle* _nh;
  tf::TransformListener* _tfListener;
  boss::StringSensorMap _sensorMap;
  boss::StringFrameMap  _frameMap;
  boss::SerializableQueue  _messageQueue;
};

#endif
