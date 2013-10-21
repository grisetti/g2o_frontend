#pragma once

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

#include "ros_sensor_publisher_handler.h"

#include "g2o_frontend/boss_logger/bimagesensor.h"

using namespace boss;
using namespace boss_logger;

class RosPinholeCameraPublisherHandler : public RosSensorPublisherHandler {
 public:
  RosPinholeCameraPublisherHandler(ros::NodeHandle *nh_, boss_logger::BaseSensor *sensor_, string topic_);
  virtual ~RosPinholeCameraPublisherHandler();
  
  virtual void publish();
  virtual void callback(PinholeImageData *data);

  inline string cameraInfoTopic() const { return _cameraInfoTopic; }
  inline ros::Publisher* cameraInfoPublisher() { return &_cameraInfoPublisher; }

  inline void setTopic(const string cameraInfoTopic_) { _cameraInfoTopic = cameraInfoTopic_; }

 protected:
  string _cameraInfoTopic;
  ros::Publisher _cameraInfoPublisher;
  
};
