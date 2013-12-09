#pragma once

#include "g2o_frontend/boss_map/image_sensor.h"
#include "ros_message_handler.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

class RosPinholeImageDataMessageHandler : public RosMessageHandler {
 public:
  RosPinholeImageDataMessageHandler(RosMessageContext* context, std::string topicName_);
  virtual ~RosPinholeImageDataMessageHandler();
  
  virtual void subscribe();
  virtual void publish(boss_map::BaseSensorData* sdata = 0);
  virtual void advertise();
  virtual bool configReady() const;
  virtual void callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info);
  
  inline boss_map::PinholeImageSensor* sensor() {return _sensor;}
  inline int pubQueueSize() { return _pubQueueSize; }

  inline void setPubQueueSize(int pubQueueSize_) { _pubQueueSize = pubQueueSize_; }
  void setSensor(boss_map::BaseSensor* sensor_);

 protected: 
  int _pubQueueSize;
  std::string _topicName;
  std::string _infoTopicName;
  ros::Publisher _pub, _infoPub;
  image_transport::ImageTransport* _imageTransport;
  image_transport::CameraSubscriber* _cameraSubscriber;
  boss_map::PinholeImageSensor* _sensor;
  sensor_msgs::CameraInfo _cameraInfo;
};
