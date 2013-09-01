#ifndef _ROS_PINHIOLE_IMAGE_DATA_MESSAGE_HANDLER_H_
#define _ROS_PINHIOLE_IMAGE_DATA_MESSAGE_HANDLER_H_
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "ros_message_handler.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

class RosPinholeImageDataMessageHandler : public RosMessageHandler{
public:
  RosPinholeImageDataMessageHandler(RosMessageContext* context, std::string topicName_  );
  virtual ~RosPinholeImageDataMessageHandler();
  virtual void subscribe();
  virtual bool configReady() const;
  virtual void callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info);
  inline boss::PinholeImageSensor* sensor() {return _sensor;}
protected:
  
  std::string _topicName;
  image_transport::ImageTransport * _imageTransport;
  image_transport::CameraSubscriber *_cameraSubscriber;
  boss::PinholeImageSensor* _sensor;
  sensor_msgs::CameraInfo _cameraInfo;
};

#endif
