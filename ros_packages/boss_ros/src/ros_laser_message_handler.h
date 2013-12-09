#pragma once

#include "g2o_frontend/boss_map/laser_sensor.h"
#include "ros_message_handler.h"
#include "sensor_msgs/LaserScan.h"

class RosLaserDataMessageHandler : public RosMessageHandler {
 public:
  RosLaserDataMessageHandler(RosMessageContext* context_, std::string topicName_);
  virtual ~RosLaserDataMessageHandler();
  
  virtual void subscribe();
  virtual void publish(boss_map::BaseSensorData* sdata = 0);
  virtual void advertise();
  virtual bool configReady() const;
  
  void callback(const sensor_msgs::LaserScanConstPtr& scan);
  inline boss_map::LaserSensor* sensor() {return _sensor;}
  inline int pubQueueSize() { return _pubQueueSize; }

  inline void setPubQueueSize(int pubQueueSize_) { _pubQueueSize = pubQueueSize_; }
  void setSensor(boss_map::BaseSensor* sensor_);

 protected:
  int _pubQueueSize;
  std::string _topicName;
  ros::Subscriber _sub;
  ros::Publisher _pub;
  boss_map::LaserSensor* _sensor;
};
