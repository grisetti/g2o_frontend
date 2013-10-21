#pragma once

#include "ros/ros.h"

#include "g2o_frontend/boss_logger/bsensor.h"

using namespace std;
using namespace boss;
using namespace boss_logger;

class RosSensorPublisherHandler {
 public:
  RosSensorPublisherHandler();
  virtual ~RosSensorPublisherHandler();
  
  virtual void publish() = 0;
  
  inline bool configured() const { return _configured; } 
  inline int sequenceID() const { return _sequenceID; } 
  inline int publishingQueueSize() const { return _publishingQueueSize; } 
  inline string topic() const { return _topic; }
  inline ros::NodeHandle* nodeHandle() { return _nh; }
  inline ros::Publisher* publisher() { return &_publisher; }
  inline boss_logger::BaseSensor* sensor() { return _sensor; }
  
  inline void setTopic(const string topic_) { _topic = topic_; }
  inline void setSequenceID(const int sequenceID_) { _sequenceID = sequenceID_; }
  inline void setPublishingQueueSize(const int publishingQueueSize_) { _publishingQueueSize = publishingQueueSize_; }
  inline void setNodeHandle(ros::NodeHandle *nh_) { _nh = nh_; }
  inline void sensor(boss_logger::BaseSensor *sensor_) { _sensor = sensor_; }

 protected:
  bool _configured;
  int _sequenceID;
  int _publishingQueueSize;
  string _topic;
  ros::NodeHandle *_nh;
  ros::Publisher _publisher;
  boss_logger::BaseSensor *_sensor; 
};
