#pragma once

#include "ros/ros.h"
#include "tf/tfMessage.h"

#include "ros_sensor_publisher_handler.h"

using namespace boss;
using namespace boss_logger;

class RosTransformPublisherHandler : public RosSensorPublisherHandler {
 public:
  RosTransformPublisherHandler(ros::NodeHandle *nh_, boss_logger::BaseSensor *sensor_, string topic_);
  virtual ~RosTransformPublisherHandler();
  
  virtual void publish();
  virtual void callback(tf::tfMessage *data);
};
