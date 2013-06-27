#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include "g2o_frontend/boss/serializer.h"

using namespace std;
using namespace boss;

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

class ImageBLOB : public BLOB {
public:
  ImageBLOB() {
  }
  virtual ~ImageBLOB() {
  }

  virtual bool read(std::istream& is) {
    return true;
  }

  virtual void write(std::ostream& os) {
    std::vector<uchar> buffer;
    bool result = imencode(".PGM", image, buffer);
    os.write((char*)(&buffer[0]),buffer.size());
    assert ( result && "tua madre");
  }

  void fromRosImage(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr ptr=cv_bridge::toCvCopy(msg, msg->encoding);
    image = ptr->image.clone();
    cerr << "image created:" << msg->header.frame_id << " " << image.cols << "x" << image.rows << endl;
  }
protected:
  cv::Mat image;
  std::string format;
};


std::vector< BLOBReference<ImageBLOB>* > references;
 
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void colorCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  //colorCameraInfo=*msg;
}

void colorImageCallback(const sensor_msgs::Image::ConstPtr& msg){
  //ImageBLOB* imageBlob = new ImageBLOB();
  //imageBlob->fromRosImage(msg);
  //references.push_back(new BLOBReference<ImageBLOB>(imageBlob));
}

void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  //depthCameraInfo = *msg;
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg){
  ImageBLOB* imageBlob = new ImageBLOB();
  imageBlob->fromRosImage(msg);
  BLOBReference<ImageBLOB>* reference = new BLOBReference<ImageBLOB>(imageBlob);
  references.push_back(reference);
  std::string madre("xtion_00_depth");
  serializer.write(madre,*reference);
  delete imageBlob;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "openni2_client");

  std::string base_topic ="xtion_00"; 
  
  ros::NodeHandle nh;

  cerr << "starting" << endl;
  listener = new tf::TransformListener;
  
  BOSS_REGISTER_BLOB(ImageBLOB)

  cerr << "subscribing" << endl;
  
  // ros::Subscriber subColorCameraInfo = nh.subscribe(base_topic+"/color/cameraInfo", 5, colorCameraInfoCallback);
  // ros::Subscriber subColorCameraImage = nh.subscribe(base_topic+"/color/image", 5, colorImageCallback);

  ros::Subscriber subDepthCameraInfo = nh.subscribe(base_topic+"/depth/cameraInfo", 5, depthCameraInfoCallback);
  ros::Subscriber subDepthCameraImage = nh.subscribe(base_topic+"/depth/image", 5, depthImageCallback);


  serializer.setFilePath("./fava.log");
  serializer.setBinaryPath("phawa<id>.png");
  
  cerr << "spinning" << endl;
  while (nh.ok()){

    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  return 0;
}
