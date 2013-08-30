#include "ros_message_context.h"

RosMessageContext::RosMessageContext(ros::NodeHandle* nh_) {
  _nh = nh_;
  _tfListener = new tf::TransformListener(*_nh, ros::Duration(30.0));
}

RosMessageContext::~RosMessageContext(){
  delete _tfListener;
}

bool RosMessageContext::getOdomPose(Eigen::Isometry3d& _trans, double time){
  bool transformFound = true;
  _tfListener->waitForTransform("/odom","/base_link",  
				ros::Time(time), ros::Duration(1.0));
  try{
    tf::StampedTransform t;
    _tfListener->lookupTransform("/odom", "/base_link", 
				 ros::Time(time), t);
    Eigen::Isometry3d transform;
    transform.translation().x()=t.getOrigin().x();
    transform.translation().y()=t.getOrigin().y();
    transform.translation().z()=t.getOrigin().z();
    Eigen::Quaterniond rot;
    rot.x()=t.getRotation().x();
    rot.y()=t.getRotation().y();
    rot.z()=t.getRotation().z();
    rot.w()=t.getRotation().w();
    transform.linear()=rot.toRotationMatrix();
    _trans = transform;
    transformFound = true;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    transformFound = false;
  }
  return transformFound;
}
