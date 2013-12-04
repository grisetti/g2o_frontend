#include "ros_imu_data_message_handler.h"

using namespace std;

RosIMUDataMessageHandler::RosIMUDataMessageHandler(RosMessageContext* context, std::string topicName_) : RosMessageHandler(context) {
  _pubQueueSize = 100;
  _topicName = topicName_;
  _sensor = 0;
}

RosIMUDataMessageHandler::~RosIMUDataMessageHandler() {
}

bool RosIMUDataMessageHandler::configReady() const {
  return _sensor;
}

void RosIMUDataMessageHandler::subscribe(){
  _sub = _context->nodeHandle()->subscribe(_topicName, 1, &RosIMUDataMessageHandler::callback, this);
  cout << "subscribing to topic: [" <<  _topicName << "]" << endl;
}

void RosIMUDataMessageHandler::setSensor(boss_map::BaseSensor* sensor_) {
  boss_map::IMUSensor* imuSensor = dynamic_cast<boss_map::IMUSensor*>(sensor_);
  if(!imuSensor) {
    cerr << "WARNING: tried to set a non IMU sensor to an IMU message handler, skipping" << endl;
    return;
  }
  _sensor = imuSensor;
}

void RosIMUDataMessageHandler::publish(boss_map::BaseSensorData* sdata) {
  if(!_sensor) {
    cerr << "WARNING: missing IMU sensor, skipping" << endl;
    return;
  }

  boss_map::IMUData *imuData = dynamic_cast<boss_map::IMUData*>(sdata);
  if(!imuData) {
    cerr << "WARNING: trying to publish non IMU data from an IMU message handler, skipping" << endl;
    return;
  }

  // Create message
  sensor_msgs::Imu imu;
  std_msgs::Header header;
  header.seq = _sequenceID;
  header.stamp = ros::Time(imuData->timestamp());
  header.frame_id = _sensor->frame()->name();
  imu.header = header;
  
  geometry_msgs::Quaternion orientation;
  orientation.x = imuData->orientation().x();
  orientation.y = imuData->orientation().y();
  orientation.z = imuData->orientation().z();  
  orientation.w = imuData->orientation().w();
  imu.orientation = orientation;
  boost::array<double, 9> orientation_covariance = {{imuData->orientationCovariance()(0, 0), imuData->orientationCovariance()(0, 1), imuData->orientationCovariance()(0, 2),
						     imuData->orientationCovariance()(1, 0), imuData->orientationCovariance()(1, 1), imuData->orientationCovariance()(1, 2),
						     imuData->orientationCovariance()(2, 0), imuData->orientationCovariance()(2, 1), imuData->orientationCovariance()(2, 2)}};
  imu.orientation_covariance = orientation_covariance;
  geometry_msgs::Vector3 angular_velocity;
  angular_velocity.x = imuData->angularVelocity().x();
  angular_velocity.y = imuData->angularVelocity().y();
  angular_velocity.z = imuData->angularVelocity().z();
  imu.angular_velocity = angular_velocity; 
  boost::array<double, 9> angular_velocity_covariance = {{imuData->angularVelocityCovariance()(0, 0), imuData->angularVelocityCovariance()(0, 1), imuData->angularVelocityCovariance()(0, 2),
							  imuData->angularVelocityCovariance()(1, 0), imuData->angularVelocityCovariance()(1, 1), imuData->angularVelocityCovariance()(1, 2),
							  imuData->angularVelocityCovariance()(2, 0), imuData->angularVelocityCovariance()(2, 1), imuData->angularVelocityCovariance()(2, 2)}};
  imu.angular_velocity_covariance = angular_velocity_covariance;
  geometry_msgs::Vector3 linear_acceleration;
  linear_acceleration.x = imuData->linearAcceleration().x();
  linear_acceleration.y = imuData->linearAcceleration().y();
  linear_acceleration.z = imuData->linearAcceleration().z();
  imu.linear_acceleration = linear_acceleration; 
  boost::array<double, 9> linear_acceleration_covariance = {{imuData->linearAccelerationCovariance()(0, 0), imuData->linearAccelerationCovariance()(0, 1), imuData->linearAccelerationCovariance()(0, 2),
							     imuData->linearAccelerationCovariance()(1, 0), imuData->linearAccelerationCovariance()(1, 1), imuData->linearAccelerationCovariance()(1, 2),
							     imuData->linearAccelerationCovariance()(2, 0), imuData->linearAccelerationCovariance()(2, 1), imuData->linearAccelerationCovariance()(2, 2)}};
  imu.linear_acceleration_covariance = linear_acceleration_covariance;

  // Publish message
  _pub.publish(imu);
  _sequenceID++;
}

void RosIMUDataMessageHandler::advertise() {
  cout << "publishing on topic: [" << _topicName << "]" << endl;
  _pub = _context->nodeHandle()->advertise<sensor_msgs::Imu>(_topicName, _pubQueueSize);
}

void RosIMUDataMessageHandler::callback(const sensor_msgs::ImuConstPtr& imu) {
  if  (! _sensor) {
    std::map<std::string, boss_map::ReferenceFrame*>::iterator it = _context->frameMap().find(imu->header.frame_id);
    if (it == _context->frameMap().end()) {
      cerr << "missing transform for frame [" << imu->header.frame_id << "], skipping" << endl;
      return;
    }
    _sensor = new boss_map::IMUSensor;
    _sensor->setTopic(_topicName);
    _sensor->setReferenceFrame(it->second);
    cerr << "created sensor for topic[" << _sensor->topic()  << "]" << endl;
    _context->sensorMap().insert(make_pair(_sensor->topic(), _sensor));
  }
  Eigen::Isometry3d robotTransform;
  if (! _context->getOdomPose(robotTransform, imu->header.stamp.toSec()) ) {
    return;
  } 
  boss_map::ReferenceFrame* newReferenceFrame = new boss_map::ReferenceFrame("robotPose", robotTransform);
  //_context->messageQueue().push_back(newReferenceFrame);
  // we get the image from ROS

  boss_map::IMUData* imuData = new boss_map::IMUData(_sensor);
  imuData->setTimestamp(imu->header.stamp.toSec());
  imuData->setRobotReferenceFrame(newReferenceFrame);
  imuData->setTopic(_sensor->topic());
  Eigen::Quaterniond orientation;
  orientation.x() = imu->orientation.x;
  orientation.y() = imu->orientation.y;
  orientation.z() = imu->orientation.z;
  orientation.w() = imu->orientation.w;
  imuData->setOrientation(orientation);
  Eigen::Matrix3d orientationCovariance;

  int k=0;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++, k++)
      orientationCovariance(i,j) = imu->orientation_covariance[k];
  imuData->setOrientationCovariance(orientationCovariance);

  Eigen::Vector3d angularVelocity;  
  angularVelocity.x() = imu->angular_velocity.x;
  angularVelocity.y() = imu->angular_velocity.y;
  angularVelocity.z() = imu->angular_velocity.z;
  imuData->setAngularVelocity(angularVelocity);

  Eigen::Matrix3d angularVelocityCovariance;
  k=0;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++, k++)
      angularVelocityCovariance(i,j) = imu->angular_velocity_covariance[k];
  imuData->setAngularVelocityCovariance(angularVelocityCovariance);

  Eigen::Vector3d linearAcceleration;
  linearAcceleration.x() = imu->linear_acceleration.x;
  linearAcceleration.y() = imu->linear_acceleration.y;
  linearAcceleration.z() = imu->linear_acceleration.z;
  imuData->setLinearAcceleration(linearAcceleration);

  Eigen::Matrix3d linearAccelerationCovariance;
  k=0;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++, k++)
      linearAccelerationCovariance(i,j) = imu->linear_acceleration_covariance[k];
  imuData->setLinearAccelerationCovariance(linearAccelerationCovariance);

  _context->messageQueue().push_back(imuData);
  //delete imageData;
  //cerr << "i";
}
