#include "ros_imu_publisher_handler.h"

#include "sensor_msgs/Imu.h"

RosImuPublisherHandler::RosImuPublisherHandler(ros::NodeHandle *nh_, boss_logger::BaseSensor *sensor_, string topic_) {
  _nh = nh_;
  _sensor = sensor_;
  _topic = topic_;
  _sequenceID = 0;
  _publishingQueueSize = 100;
}

RosImuPublisherHandler::~RosImuPublisherHandler() {}

void RosImuPublisherHandler::publish() {
  cout << "Imu sensor publishing data on topic " << _topic << endl;
  _publisher = _nh->advertise<sensor_msgs::Imu>(_topic, _publishingQueueSize);
  _configured = true;
}

void RosImuPublisherHandler::callback(IMUData *data) {
  IMUSensor *imuSensor = dynamic_cast<IMUSensor*>(_sensor);
  if(!imuSensor) {
    cerr << "WARNING: imu publisher handler has a sensor which is not of type imu, skipping" << endl;
    return;
  }

  // Create message
  sensor_msgs::Imu imu;
  std_msgs::Header header;
  header.seq = _sequenceID;
  header.stamp = ros::Time(data->timestamp());
  header.frame_id = imuSensor->frame()->name();
  imu.header = header;
  
  geometry_msgs::Quaternion orientation;
  orientation.x = data->orientation().x();
  orientation.y = data->orientation().y();
  orientation.z = data->orientation().z();  
  orientation.w = data->orientation().w();
  imu.orientation = orientation;
  boost::array<int, 9> orientation_covariance = {{data->orientationCovariance()(0, 0), data->orientationCovariance()(0, 1), data->orientationCovariance()(0, 2),
						  data->orientationCovariance()(1, 0), data->orientationCovariance()(1, 1), data->orientationCovariance()(1, 2),
						  data->orientationCovariance()(2, 0), data->orientationCovariance()(2, 1), data->orientationCovariance()(2, 2)}};
  imu.orientation_covariance = orientation_covariance;
  geometry_msgs::Vector3 angular_velocity;
  angular_velocity.x = data->angularVelocity().x();
  angular_velocity.y = data->angularVelocity().y();
  angular_velocity.z = data->angularVelocity().z();
  imu.angular_velocity = angular_velocity; 
  boost::array<int, 9> angular_velocity_covariance = {{data->angularVelocityCovariance()(0, 0), data->angularVelocityCovariance()(0, 1), data->angularVelocityCovariance()(0, 2),
						       data->angularVelocityCovariance()(1, 0), data->angularVelocityCovariance()(1, 1), data->angularVelocityCovariance()(1, 2),
						       data->angularVelocityCovariance()(2, 0), data->angularVelocityCovariance()(2, 1), data->angularVelocityCovariance()(2, 2)}};
  imu.angular_velocity_covariance = angular_velocity_covariance;
  geometry_msgs::Vector3 linear_acceleration;
  linear_acceleration.x = data->linearAcceleration().x();
  linear_acceleration.y = data->linearAcceleration().y();
  linear_acceleration.z = data->linearAcceleration().z();
  imu.linear_acceleration = linear_acceleration; 
  boost::array<int, 9> linear_acceleration_covariance = {{data->linearAccelerationCovariance()(0, 0), data->linearAccelerationCovariance()(0, 1), data->linearAccelerationCovariance()(0, 2),
							  data->linearAccelerationCovariance()(1, 0), data->linearAccelerationCovariance()(1, 1), data->linearAccelerationCovariance()(1, 2),
							  data->linearAccelerationCovariance()(2, 0), data->linearAccelerationCovariance()(2, 1), data->linearAccelerationCovariance()(2, 2)}};
  imu.linear_acceleration_covariance = linear_acceleration_covariance;

  // Publish message
  _publisher.publish(imu);
  _sequenceID++;
}
