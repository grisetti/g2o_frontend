#pragma once

#include "ros/ros.h"
#include "g2o_frontend/boss_map/sensor.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "tf/transform_listener.h"

using namespace std;
using namespace boss_map;

class RosMessageHandler;
class RosTransformMessageHandler;
class RosMessageContext : public boss_map::RobotConfiguration {
public:
  RosMessageContext(ros::NodeHandle* nh_);
  RosMessageContext(ros::NodeHandle* nh_, boss_map::RobotConfiguration* robotConfiguration);
  virtual ~RosMessageContext();

  inline ros::NodeHandle* nodeHandle() {return _nh;}
  boss_map::StringSensorMap& sensorMap() {return _sensorMap;}
  boss_map::StringReferenceFrameMap&  frameMap() {return _frameMap;}
  boss_map::SerializableQueue& messageQueue() {return _messageQueue;}
  bool addHandler(const std::string& type, const std::string& topic);
  RosMessageHandler* handler(const std::string topic);
  bool configReady() const;
  void initSubscribers();
  void initPublishers();
  void updateOdomReferenceFrame(boss_map::ReferenceFrame* newOdomReRerenceFrame);

  bool getOdomPose(Eigen::Isometry3d& t, double time);
  inline void setOdomReferenceFrameId(const std::string odomReferenceFrameId_) {_odomReferenceFrameId = odomReferenceFrameId_;}
  inline const std::string& odomReferenceFrameId() const {return _odomReferenceFrameId;}
  inline std::map<std::string, RosMessageHandler*> handlers() { return _handlers; }
  inline RosTransformMessageHandler* transformHandler() { return _transformHandler; } 
  
protected:
  ros::NodeHandle* _nh;
  tf::TransformListener* _tfListener;
  RosTransformMessageHandler* _transformHandler;
  boss_map::SerializableQueue  _messageQueue;
  std::string _odomReferenceFrameId;
  std::map<std::string, RosMessageHandler*> _handlers;  
};
