#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <g2o/core/hyper_graph.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o_frontend/sensor_data/priority_synchronous_data_queue.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <g2o_frontend/sensor_data/sensor_data.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>



#include <g2o/stuff/command_args.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <iostream>

using namespace std;
using namespace g2o;

std::string intensityName;
std::string depthTopic = "/camera/depth/image_raw";
int sequenceNumber =0;

void sensorCallback(const sensor_msgs::ImageConstPtr& raw_depth_image) {
  ROS_WARN_ONCE_NAMED("eval", "First RGBD-Data Received");
  // It's where we put the image that we will show
  cv_bridge::CvImagePtr depthImage;
  cv_bridge::CvImagePtr intensityImage;
  // Convert the gray and depth image in input to a OpenCV image type
  try
  {
    depthImage = cv_bridge::toCvCopy(raw_depth_image, raw_depth_image->encoding);
  }
  catch (cv_bridge::Exception& ex)
  {
  	ROS_ERROR("cv_bridge exception: %s", ex.what());
    return;
  }
  cv::Mat* depthImagePtr = new cv::Mat(depthImage->image);
  char buf [1024];
  sprintf(buf,"%s-%07d.pgm", intensityName.c_str(),sequenceNumber);
  cv::imwrite(buf, *depthImagePtr);
  sequenceNumber++;
  delete depthImagePtr;
}

int main(int argc, char** argv) {
  // Initialize ros
  g2o::CommandArgs arg;
  arg.paramLeftOver("image-prefix", intensityName , "boh", "prefix of the images written", true);
  arg.parseArgs(argc, argv);
  if (intensityName==""){
    cerr << "an output prefix should be specified" << endl;
    return 0;
  }

  ros::init(argc, argv, "bag2img");
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image>* _depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, depthTopic.c_str(), 1);
  _depth_sub->registerCallback(boost::bind(&sensorCallback, _1));
  ros::spin();
}
