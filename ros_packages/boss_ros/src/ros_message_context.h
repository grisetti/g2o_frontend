#pragma once

#include "ros/ros.h"
#include "g2o_frontend/boss_logger/bsensor.h"
#include "g2o_frontend/boss_logger/brobot_configuration.h"
#include "tf/transform_listener.h"

using namespace std;
using namespace boss_logger;

class RosMessageHandler;
class RosTransformMessageHandler;
class RosMessageContext : public boss_logger::RobotConfiguration {
public:
  RosMessageContext(ros::NodeHandle* nh_);
  RosMessageContext(ros::NodeHandle* nh_, boss_logger::RobotConfiguration* robotConfiguration);
  virtual ~RosMessageContext();

  inline ros::NodeHandle* nodeHandle() {return _nh;}
  boss_logger::StringSensorMap& sensorMap() {return _sensorMap;}
  boss_logger::StringReferenceFrameMap&  frameMap() {return _frameMap;}
  boss_logger::SerializableQueue& messageQueue() {return _messageQueue;}
  bool addHandler(const std::string& type, const std::string& topic);
  RosMessageHandler* handler(const std::string topic);
  bool configReady() const;
  void initSubscribers();
  void initPublishers();
  void updateOdomReferenceFrame(boss_logger::ReferenceFrame* newOdomReRerenceFrame);

  bool getOdomPose(Eigen::Isometry3d& t, double time);
  inline void setOdomReferenceFrameId(const std::string odomReferenceFrameId_) {_odomReferenceFrameId = odomReferenceFrameId_;}
  inline const std::string& odomReferenceFrameId() const {return _odomReferenceFrameId;}
  inline std::map<std::string, RosMessageHandler*> handlers() { return _handlers; }
  inline RosTransformMessageHandler* transformHandler() { return _transformHandler; } 
  
protected:
  ros::NodeHandle* _nh;
  tf::TransformListener* _tfListener;
  RosTransformMessageHandler* _transformHandler;
  boss_logger::SerializableQueue  _messageQueue;
  std::string _odomReferenceFrameId;
  std::map<std::string, RosMessageHandler*> _handlers;  
};
