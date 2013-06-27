#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "tf/transform_listener.h"

using namespace std;

tf::TransformListener* listener=0;


sensor_msgs::CameraInfo depthCameraInfo, colorCameraInfo;
sensor_msgs::Image      depthImage, colorImage;
bool depthImageReceived, colorImageReceived;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void colorCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  colorCameraInfo=*msg;
}

void colorImageCallback(const sensor_msgs::Image::ConstPtr& msg){
  if(msg->encoding != "rgb8")
    cerr << __PRETTY_FUNCTION__ << ": encoding error" << endl;
  colorImage=*msg;
  colorImageReceived = true;
}

void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  depthCameraInfo = *msg;
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg){
  if(msg->encoding != "mono16")
    cerr << __PRETTY_FUNCTION__ << ": encoding error" << endl;
  depthImage = *msg;
  depthImageReceived = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "openni2_client");

  std::string base_topic ="xtion_00"; 
  
  ros::NodeHandle nh;

  cerr << "starting" << endl;
  listener = new tf::TransformListener;
  
  cerr << "subscribing" << endl;
  
  ros::Subscriber subColorCameraInfo = nh.subscribe(base_topic+"/color/cameraInfo", 5, colorCameraInfoCallback);
  ros::Subscriber subColorCameraImage = nh.subscribe(base_topic+"/color/image", 5, colorImageCallback);

  ros::Subscriber subDepthCameraInfo = nh.subscribe(base_topic+"/depth/cameraInfo", 5, depthCameraInfoCallback);
  ros::Subscriber subDepthCameraImage = nh.subscribe(base_topic+"/depth/image", 5, depthImageCallback);

  cerr << "spinning" << endl;
  while (nh.ok()){
    if (depthImageReceived) {
      depthImageReceived = false;
      cerr << "D";
      cerr << depthImage.header.stamp << endl;
      
    }

    if (colorImageReceived) {
      colorImageReceived = false;
      cerr << "C";
      cerr << colorImage.header.stamp << endl;
    }

    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  return 0;
}
