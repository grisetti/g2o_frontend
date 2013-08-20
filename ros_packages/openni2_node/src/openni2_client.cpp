#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"
#include <sstream>
#include <stdexcept>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"

using namespace std;
using namespace boss;

ImageSensor* imageSensor=0;
Serializer serializer;
tf::TransformListener* listener=0;
/*
  cv_bridge::CvImagePtr depthImage;
  cv_bridge::CvImagePtr intensityImage;
  // Convert the gray and depth image in input to a OpenCV image type
  try
  {
    intensityImage = cv_bridge::toCvCopy(raw_intensity_image, raw_intensity_image->encoding);
    depthImage = cv_bridge::toCvCopy(raw_depth_image, raw_depth_image->encoding);
  }
  catch (cv_bridge::Exception& ex)
  {
  	ROS_ERROR("cv_bridge exception: %s", ex.what());
    return;
  }
  cv::Mat* intensityImagePtr = new cv::Mat(intensityImage->image);
  cv::Mat* depthImagePtr = new cv::Mat(depthImage->image);
*/

  class RosImageBLOB: public ImageBLOB {
  public:
    // enum Format {mono8=0x0, mono16=0x1, rgb8=0x2};
    // ImageBLOB();
    // virtual const std::string& extension();
    // void setExtension(const std::string& extension_) {_extension = extension_;}
    // void resize(int width, int height, Format format);
    // Format format() const { return _format; }
    // virtual bool read(std::istream& is);
    // virtual void write(std::ostream& os);
    void fromRosImage(const sensor_msgs::Image::ConstPtr& msg);
  protected:
    // inline cv::Mat& cvImage() {return _image;}
    // const cv::Mat& cvImage() const {return _image;}
    // std::string _extension;
    // cv::Mat _image;
    // Format _format;
  };


void RosImageBLOB::fromRosImage(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr ptr=cv_bridge::toCvCopy(msg, msg->encoding);
    _image = ptr->image.clone();
    switch (_image.type()){
    case CV_8UC1: 
      _format = mono8;
      _extension = "pgm";
      break;
    case CV_16UC1: 
      _format = mono16;
      _extension = "pgm";
      break;
    case CV_8UC3: 
      _format = rgb8;
      _extension = "pbm";
      break;
    }
    //cerr << "image created:" << msg->header.frame_id << " " << _image.cols << "x" << _image.rows <<  " extension: " << _extension << endl;
  }

sensor_msgs::CameraInfo colorCameraInfo, depthCameraInfo;
std::vector< BLOBReference<RosImageBLOB>* > references;
 
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void colorCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  colorCameraInfo = *msg;
}

void colorImageCallback(const sensor_msgs::Image::ConstPtr& msg){
  RosImageBLOB* imageBlob = new RosImageBLOB();
  imageBlob->fromRosImage(msg);
  Image img;
  img.setSensor(imageSensor);
  img.imageBlob().set(imageBlob);
  size_t k=0;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      img._cameraMatrix(i,j)=colorCameraInfo.K[k++];
  img._distortionParameters.resize(1,colorCameraInfo.D.size());
  for (k=0; k<colorCameraInfo.D.size(); k++)
    img._distortionParameters(0,k)=colorCameraInfo.D[k];
  std::string madre("xtion_00_color");
  serializer.write(madre,img);
  delete imageBlob;
}

void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  depthCameraInfo = *msg;
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg){
  RosImageBLOB* imageBlob = new RosImageBLOB();
  imageBlob->fromRosImage(msg);
  Image img;
  img.setSensor(imageSensor);
  img.imageBlob().set(imageBlob);
  size_t k=0;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      img._cameraMatrix(i,j)=depthCameraInfo.K[k++];
  img._distortionParameters.resize(1,depthCameraInfo.D.size());
  for (k=0; k<depthCameraInfo.D.size(); k++)
    img._distortionParameters(0,k)=depthCameraInfo.D[k];
  std::string madre("xtion_00_depth");
  serializer.write(madre,img);
  delete imageBlob;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "openni2_client");

  std::string base_topic ="xtion_00"; 
  serializer.setFilePath("./fava.log");
  serializer.setBinaryPath("fava<id>.<ext>");
  imageSensor= new ImageSensor();
  imageSensor->setTopic(base_topic);
  serializer.write(base_topic, *imageSensor);
 
  ros::NodeHandle nh;

  cerr << "starting" << endl;
  listener = new tf::TransformListener;
  cerr << "subscribing" << endl;
  
  // ros::Subscriber subColorCameraInfo = nh.subscribe(base_topic+"/color/camera_info", 5, colorCameraInfoCallback);
  // ros::Subscriber subColorCameraImage = nh.subscribe(base_topic+"/color/image", 5, colorImageCallback);
  
  ros::Subscriber subDepthCameraInfo = nh.subscribe(base_topic+"/depth/camera_info", 5, depthCameraInfoCallback);
  ros::Subscriber subDepthCameraImage = nh.subscribe(base_topic+"/depth/image", 5, depthImageCallback);

  cerr << "spinning" << endl;
  while (nh.ok()){

    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  return 0;
}
