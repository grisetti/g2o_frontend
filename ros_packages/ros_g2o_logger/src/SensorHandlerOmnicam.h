
#ifndef SENSORHANDLEROMNICAM_H
#define SENSORHANDLEROMNICAM_H

#include "g2o_frontend/sensor_data/sensor_handler.h"
#include "g2o_frontend/sensor_data/sensor_omnicam.h"
#include "g2o_frontend/sensor_data/priority_synchronous_data_queue.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

class SensorHandlerOmnicam : public SensorHandler
{

public:
  SensorHandlerOmnicam(tf::TransformListener* tfListener_);
  virtual ~SensorHandlerOmnicam();
  
  virtual Sensor* getSensor();
  bool setQueue(PriorityDataQueue* queue_);
  bool setSensor(Sensor* sensor_s);
  void setNodeHandler(ros::NodeHandle* nptr_);
  ros::NodeHandle* getNodeHandler() { return _nptr; };
  virtual void registerCallback();
  PriorityDataQueue* getQueue() {return _queue;};
  void callback(const sensor_msgs::ImageConstPtr& raw_image);
 protected:
  void _sensorCallback(const sensor_msgs::ImageConstPtr& raw_image);
  void _calibrationCallback(const sensor_msgs::ImageConstPtr& raw_image);
  
  ros::Subscriber * _omnicam_sub;
  ros::NodeHandle* _nptr;
  tf::TransformListener* _tfListener;
  bool _isCalibrated;
};

#endif // SENSORHANDLEROMNICAM_H
