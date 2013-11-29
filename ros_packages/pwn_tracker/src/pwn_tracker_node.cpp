#include "pwn_tracker_ros.h"
#include <fstream>
#include <iostream>

using namespace std;
using namespace pwn;
using namespace boss;
using namespace pwn_tracker;

int main(int argc, char** argv){
  if (argc<3) {
    cerr << " You should provide a config file AND an oputput file" << endl;
    return 0;
  }

  Aligner* aligner;
  DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);

  ros::init(argc, argv, "pwn_tracker");
  ros::NodeHandle nh;
  tf::TransformListener* tfListener = new tf::TransformListener(nh, ros::Duration(30.0));
  tf::TransformBroadcaster* tfBroadcaster = new tf::TransformBroadcaster();

  std::string _topic = "/camera/depth_registered/image_rect_raw";
  boss_map::MapManager* manager = new boss_map::MapManager();

  PwnTrackerRos* tracker = new PwnTrackerRos(nh, tfListener, tfBroadcaster, _topic, argv[2],
					     aligner, converter, manager);
  // hand held camera
  // tracker->_base_frame_id = "/camera_link";
  // tracker->_scale = 4;

  // catacombs
  // tracker->_base_frame_id = "/base_link";
  // tracker->_odom_frame_id = "/odom";
  // tracker->_scale = 2;

  // giorgio home  
  tracker->_base_frame_id = "/base_link";
  tracker->_odom_frame_id = "/odom";
  tracker->_scale = 4;

  tracker->init();
  
  tracker->subscribe();
  while (ros::ok()){
    ros::spinOnce();
  }
}
