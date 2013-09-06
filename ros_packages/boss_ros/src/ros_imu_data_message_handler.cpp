#include "ros_imu_data_message_handler.h"

using namespace std;

RosIMUDataMessageHandler::RosIMUDataMessageHandler(RosMessageContext* context, std::string topicName_): RosMessageHandler(context){
    _topicName = topicName_;
    _sensor = 0;
  }

RosIMUDataMessageHandler::~RosIMUDataMessageHandler() {
}

bool RosIMUDataMessageHandler::configReady() const{
  return _sensor;
}

void RosIMUDataMessageHandler::subscribe(){
  _sub= _context->nodeHandle()->subscribe(_topicName, 1, &RosIMUDataMessageHandler::callback, this);
  cerr << "subscribed topics: [" <<  _topicName << "]" << endl;
}


void RosIMUDataMessageHandler::callback(const sensor_msgs::ImuConstPtr& imu){
  if  (! _sensor) {
    std::map<std::string, boss::ReferenceFrame*>::iterator it = _context->frameMap().find(imu->header.frame_id);
    if (it == _context->frameMap().end()) {
      cerr << "missing transform for frame [" << imu->header.frame_id << "], skipping" << endl;
      return;
    }
    _sensor = new boss::IMUSensor;
    _sensor->setTopic(_topicName);
    _sensor->setReferenceFrame(it->second);
    cerr << "created sensor for topic[" << _sensor->topic()  << "]" << endl;
    _context->sensorMap().insert(make_pair(_sensor->topic(), _sensor));
  }
  Eigen::Isometry3d robotTransform;
  if (! _context->getOdomPose(robotTransform, imu->header.stamp.toSec()) ){
    return;
  } 
  boss::ReferenceFrame* newReferenceFrame = new boss::ReferenceFrame("robotPose", robotTransform);
  //_context->messageQueue().push_back(newReferenceFrame);
  // we get the image from ROS

  boss::IMUData* imuData = new boss::IMUData(_sensor);
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
  imuData->setAngularVelocity(linearAcceleration);

  Eigen::Matrix3d linearAccelerationCovariance;
  k=0;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++, k++)
      linearAccelerationCovariance(i,j) = imu->linear_acceleration_covariance[k];
  imuData->setAngularVelocityCovariance(angularVelocityCovariance);

  _context->messageQueue().push_back(imuData);
  //delete imageData;
  //cerr << "i";
}
