#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include "ros_sensor_publisher_handler.h"

#include "g2o_frontend/boss_logger/bimusensor.h"

using namespace boss;
using namespace boss_logger;

class RosImuPublisherHandler : public RosSensorPublisherHandler {
 public:
  RosImuPublisherHandler(ros::NodeHandle *nh_, boss_logger::BaseSensor *sensor_, string topic_);
  virtual ~RosImuPublisherHandler();
  
  virtual void publish();
  virtual void callback(IMUData *data);
};
