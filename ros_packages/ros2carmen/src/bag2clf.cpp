#include <fstream>
#include <iomanip>

#include "bag2clf.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
using namespace std;

ROS2CarmenBagReader::ROS2CarmenBagReader(std::string odom, std::string scan, std::string tf) :
  topicOdom(odom),
  topicScan(scan),
  topicTf(tf){
}

ROS2CarmenBagReader::~ROS2CarmenBagReader() {}

void ROS2CarmenBagReader::readBag(std::string filename, std::string carmenfilename) {
  std::vector<std::string> topics;
  topics.push_back(topicScan);
  topics.push_back(topicOdom);
  topics.push_back(topicTf);

  rosbag::Bag bag(filename);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  rosbag::View::iterator it = view.begin();

  // bool haveScan, haveOdom = false;
  // haveScan = haveOdom = false;
  // ros::Time scan;

  // Eigen::Quaterniond qLaser(1.0, 0.000, 0.000, 0.000);
  // Eigen::Vector3d tLaser(0.100, 0.000, 0.250);
  // g2o::SE3Quat baseToLaserTransform(qLaser, tLaser);
  // g2o::SE3Quat odomPose;
  ofstream os(carmenfilename.c_str());
  std::list<sensor_msgs::LaserScan::ConstPtr> pendingLasers;
  cerr << "BAG OPENED" << endl;
  while (it != view.end()) {
    std::string topic = it->getTopic();
    if (topic == topicOdom){
      nav_msgs::Odometry::ConstPtr odom=it->instantiate<nav_msgs::Odometry>();
      //printf("BagReader: odom. %d\n", odom->header.seq);
    } else if (topic == topicScan) {
      sensor_msgs::LaserScan::ConstPtr scan=it->instantiate<sensor_msgs::LaserScan>();
      //printf("BagReader: scan. %d\n", scan->header.seq);
      pendingLasers.push_back(scan);
    } else if (topic == topicTf){
      tf::tfMessage::ConstPtr tfMessage=it->instantiate<tf::tfMessage>();
      //printf("BagReader: tf ...");
      for (unsigned int i = 0; i < tfMessage->transforms.size(); i++) {
	tf::StampedTransform trans;
	tf::transformStampedMsgToTF(tfMessage->transforms[i], trans);
	try {
	  const std::map<std::string, std::string>* msg_header_map = tfMessage->__connection_header.get();
	  std::string authority;
	  std::map<std::string, std::string>::const_iterator it = msg_header_map->find("callerid");
	  if (it == msg_header_map->end()) {
	    ROS_WARN("Message recieved without callerid");
	    authority = "no callerid";
	  } else {
	    authority = it->second;
	  }
	  _transformer.setTransform(trans, authority);
	  //printf("Done \n");
	}
	catch (tf::TransformException& ex) {
	    ///\todo Use error reporting
	    std::string temp = ex.what();
	    ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", tfMessage->transforms[i].child_frame_id.c_str(), tfMessage->transforms[i].header.frame_id.c_str(), temp.c_str());
	}
	//printf("Done \n");
      }

      //printf("try to process ");
      bool tryToProcess = true;
      while ( tryToProcess && ! pendingLasers.empty()) {
	//cerr << "pendingLasers = " << pendingLasers.size() << endl;
	sensor_msgs::LaserScan::ConstPtr scan = *pendingLasers.begin();
	std::string frame_id = scan->header.frame_id;
	tf::StampedTransform laserPose;
	tf::StampedTransform odomPose;
	const ros::Time& scanTime=scan->header.stamp;
	std::string error;
	bool canTransform = _transformer.canTransform (frame_id, "odom", scanTime, &error);
	if (! canTransform ) {
	  //cerr << "cannot transform, error= " << frame_id << " " << error << endl;
	  const std::string pastExtrapolationError("Lookup would require extrapolation into the past");
	  const std::string futureExtrapolationError("Lookup would require extrapolation into the future");
	  const std::string timeExtrapolationError("Lookup would require extrapolation at time");

	  // extrapolation in the past: discard the scan
	  if (error.substr(0,pastExtrapolationError.length()) == pastExtrapolationError) {
	    cerr << "past extrapolation, discarding scan " << endl;
	    pendingLasers.pop_front();
	  } else if (error.substr(0,timeExtrapolationError.length()) == timeExtrapolationError) {
	    cerr << "dunno extrapolation, discarding scan " << endl;
	    pendingLasers.pop_front();
	  } else if (error.substr(0,futureExtrapolationError.length()) == futureExtrapolationError) {
	    //cerr << "future extrapolation, waiting" << endl;
	    tryToProcess=false;
	  } else {
	    pendingLasers.pop_front();
	  }

	} else {
	  cerr << ".";
	  try{
	    _transformer.lookupTransform 	("odom",
						 frame_id,
						 scanTime,
						 laserPose);
	    _transformer.lookupTransform 	("odom",
						 "base_link",
						 scanTime,
						 odomPose);
	    pendingLasers.pop_front();
	    //cerr << "Processing scan";
	    
	    
	    double odomX, odomY, odomTheta;
	    double laserX, laserY, laserTheta;
	    odomX=odomPose.getOrigin().x();
	    odomY=odomPose.getOrigin().y();

	    double yaw,pitch,roll;
	    tf::Matrix3x3 mat =  odomPose.getBasis();
	    mat.getEulerZYX(yaw, pitch, roll);
	    odomTheta = yaw;

	    laserX=laserPose.getOrigin().x();
	    laserY=laserPose.getOrigin().y();
	    mat =  laserPose.getBasis();
	    mat.getEulerZYX(yaw, pitch, roll);
	    laserTheta = yaw;

	    os.setf(ios::fixed);
	    os << setprecision(6);
	    os << "ROBOTLASER1 " << 0 << " "
	       << scan->angle_min << " " 
	       << scan->angle_max - scan->angle_min << " "
	       << scan->angle_increment << " " 
	       << scan->range_max << " "
	       << 0.1 << " " // accuracy
	       << 0 << " " // remission mode
	       <<  scan->ranges.size() << " ";
	    os << setprecision(2);
	    for (uint i=0; i<scan->ranges.size(); i++){
	      if (scan->ranges[i]<scan->range_min)
		os  << scan->range_max;
	      else 
		os << scan->ranges[i];
	      os << " ";
	    }
	    os << " 0 "; // 0 remissions
	    os.setf(ios::fixed);
	    os << setprecision(6);
    
	    os << laserX << " " << laserY << " " << laserTheta << " ";
	    os << odomX << " " << odomY << " " << odomTheta << " ";
	    os << 0  << " " // laser tv
	       << 0  << " " // laser rv
	       << 0  << " " // forward_safety_dist
	       << 0  << " " // side_safty_dist
	       << 0  << " "; // turn_axis
									
	    char timeString[50];
	    sprintf(timeString, "%d.%08d", scan->header.stamp.sec, scan->header.stamp.nsec);
	    os << timeString << " hostname ";
	    os << timeString << endl;
	    os << flush;
	    
	  }
	  catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what());
	      pendingLasers.pop_front();
	      sleep(1);
	  }
	}
      }
    }
    it++;
  }
  printf("rosbag file completed.\n");

  bag.close();
}

int main(int argc, char** argv){
  if (argc<3){
    cout << "usage: " << argv[0] << " <ros bag> <carmen filename>"; 
    return 0;
  }
  ROS2CarmenBagReader reader;
  reader.readBag(argv[1], argv[2]);
  return 0;
}
