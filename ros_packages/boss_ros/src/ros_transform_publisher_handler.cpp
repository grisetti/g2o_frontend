#include "ros_transform_publisher_handler.h"

RosTransformPublisherHandler::RosTransformPublisherHandler(ros::NodeHandle *nh_, boss_logger::BaseSensor *sensor_, string topic_) {
  _nh = nh_;
  _sensor = sensor_;
  _topic = topic_;
  _sequenceID = 0;
  _publishingQueueSize = 100;
}

RosTransformPublisherHandler::~RosTransformPublisherHandler() {}

void RosTransformPublisherHandler::publish() {
  cout << "Transform sensor publishing data on topic " << _topic << endl;
  _publisher = _nh->advertise<tf::tfMessage>(_topic, _publishingQueueSize);
  _configured = true;
}

void RosTransformPublisherHandler::callback(tf::tfMessage *data) {
  
  // Create message
  tf::tfMessage tfMessage;
  std_msgs::Header header;
  header.seq = _sequenceID;
  header.stamp = ros::Time(data->timestamp());
  header.frame_id = laserSensor->frame()->name();

  
  // Publish message
  _publisher.publish(laserScan);
  _sequenceID++;
}
