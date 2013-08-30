#ifndef _ROS_PINHIOLE_IMAGE_DATA_MESSAGE_HANDLER_H_
#define _ROS_PINHIOLE_IMAGE_DATA_MESSAGE_HANDLER_H_
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "ros_message_handler.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class RosPinholeImageDataMessageHandler : public RosMessageHandler{
public:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;

  RosPinholeImageDataMessageHandler(RosMessageContext* context, std::string topicName_, std::string imageSubtopic, std::string infoSubtopic);
  virtual ~RosPinholeImageDataMessageHandler();
  virtual void subscribe();
  virtual void callback(sensor_msgs::Image::ConstPtr img, sensor_msgs::CameraInfo::ConstPtr info);
  inline boss::PinholeImageSensor* sensor() {return _sensor;}
protected:
  message_filters::Subscriber<sensor_msgs::Image>* _imageSub;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* _infoSub;
  message_filters::Synchronizer<MySyncPolicy>* _sync;
  
  std::string _topicName;
  std::string _imageTopicName;
  std::string _cameraInfoTopicName;

  boss::PinholeImageSensor* _sensor;
  sensor_msgs::CameraInfo _cameraInfo;
};

#endif
