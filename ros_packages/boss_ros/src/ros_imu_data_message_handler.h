#pragma once

#include "g2o_frontend/boss_map/imu_sensor.h"
#include "ros_message_handler.h"
#include "sensor_msgs/Imu.h"

class RosIMUDataMessageHandler : public RosMessageHandler {
 public:
  RosIMUDataMessageHandler(RosMessageContext* context, std::string topicName_);
  virtual ~RosIMUDataMessageHandler();
  
  virtual void subscribe();
  virtual void publish(boss_map::BaseSensorData* sdata = 0);
  virtual void advertise();
  virtual bool configReady() const;
  
  void callback(const sensor_msgs::ImuConstPtr& imu);
  inline boss_map::IMUSensor* sensor() {return _sensor;}
  inline int pubQueueSize() { return _pubQueueSize; }

  inline void setPubQueueSize(int pubQueueSize_) { _pubQueueSize = pubQueueSize_; }
  void setSensor(boss_map::BaseSensor* sensor_);

 
 protected:
  int _pubQueueSize;
  std::string _topicName;
  ros::Subscriber _sub;
  ros::Publisher _pub;
  boss_map::IMUSensor* _sensor;
};
