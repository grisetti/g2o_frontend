#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "ros_sensor_publisher_handler.h"

#include "g2o_frontend/boss_logger/blasersensor.h"

using namespace boss;
using namespace boss_logger;

class RosLaserPublisherHandler : public RosSensorPublisherHandler {
 public:
  RosLaserPublisherHandler(ros::NodeHandle *nh_, boss_logger::BaseSensor *sensor_, string topic_);
  virtual ~RosLaserPublisherHandler();
  
  virtual void publish();
  virtual void callback(LaserData *data);
};
