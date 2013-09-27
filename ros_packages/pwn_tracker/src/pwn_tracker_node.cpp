#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>


#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "highgui.h"
#include <boost/bind.hpp>
#include <fstream>
#include <iostream>
#include "pwn_tracker.h"
using namespace std;
using namespace pwn;
using namespace boss;
using namespace pwn_tracker;

int main(int argc, char** argv){
  if (argc<3) {
    cerr << " u should provide a config file AND an oputput file" << endl;
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
  PwnTracker* tracker = new PwnTracker(nh, tfListener, tfBroadcaster, _topic, argv[2]);
  tracker->_aligner = aligner;
  tracker->_converter = converter;
  tracker->_base_frame_id = "/camera_link";
  boss_map::MapManager* manager = new boss_map::MapManager();
  tracker->manager = manager;
  tracker->subscribe();
  while (ros::ok()){
    ros::spinOnce();
  }
}
