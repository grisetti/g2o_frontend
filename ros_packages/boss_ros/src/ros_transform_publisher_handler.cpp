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
  TransformSensor *laserSensor = dynamic_cast<TransformSensor*>(_sensor);
  if(!laserSensor) {
    cerr << "WARNING: laser publisher handler has a sensor which is not of type laser, skipping" << endl;
    return;
  }

  // Create message
  sensor_msgs::TransformScan laserScan;
  std_msgs::Header header;
  header.seq = _sequenceID;
  header.stamp = ros::Time(data->timestamp());
  header.frame_id = laserSensor->frame()->name();
  laserScan.header = header;
  laserScan.angle_min = -1.57079637051f;
  laserScan.angle_max = 1.56643295288f;
  laserScan.angle_increment = 0.00436332309619f; 
  laserScan.time_increment = 1.73611115315e-05;
  laserScan.scan_time = 0.0250000003725;
  laserScan.range_min = laserSensor->minRange();
  laserScan.range_max = laserSensor->maxRange();
  laserScan.ranges = data->ranges();
  laserScan.intensities = data->remissions();

  // Publish message
  _publisher.publish(laserScan);
  _sequenceID++;
}
