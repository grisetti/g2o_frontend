#include <fstream>
#include <iomanip>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include <math.h>
#include <sys/poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/thread/thread.hpp>

#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "tf/transform_listener.h"

/*#include <dis_stage/Fiducial.h>
#include <dis_stage/FiducialArray.h> */
//#include <LaserDetector/SemanticInfo.h>


#define KEYCODE_X_CAP 0x58
#define KEYCODE_X 0x78
/*
 rosrun ros2carmen clf_writer \
 _odom_frame_id:=/robot_0/odom \
 _laser_frame_id:=/robot_0/base_laser_link  \
 _base_link_frame_id:=/robot_0/base_laser_link \
 _laser_topic:=/robot_0/base_scan \
 _fiducial_topic:=/robot_0/fiducial
*/

using namespace std;

tf::TransformListener* listener=0;
std::string odom_frame_id="odom";
std::string laser_frame_id="laser"; // If you perform experiments with real robots decommented this line
//std::string laser_frame_id="base_laser_link"; // If you simulate on Stage, decomment this line
std::string base_link_frame_id="base_link";
std::string laser_topic="scan"; // If you perform experiments with real robots decommented this line
//std::string laser_topic="base_scan"; // If you simulate on Stage, decomment this line
std::string kinect_topic="LaserPoseROS";
std::string fiducial_topic="fiducial";
std::string carmen_filename="out.clf";
std::ofstream os;

double lastOdomX;
double lastOdomY;
double lastOdomTh;

double uglyOdomX;
double uglyOdomY;
double uglyOdomTh;


int kfd = 0;
struct termios cooked, raw;
bool done;


void exitChar() {
  char c;
  bool dirty = false;
    
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
    
  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
    
  for(;;)
    {
      boost::this_thread::interruption_point();
        
      // get the next event from the keyboard
      int num;
        
      if ((num = poll(&ufd, 1, 250)) < 0)
        {
	  perror("poll():");
	  return;
        }
      else if(num > 0)
        {
	  if(read(kfd, &c, 1) < 0)
            {
	      perror("read():");
	      return;
            }
        }
      else
        {
	  if (dirty == true)
            {
	      dirty = false;
            }
            
	  continue;
        }
        
      switch(c)
        {
	case KEYCODE_X_CAP:
	  int i;
	  if((i = system("kill -9 `pidof clf_writer`")) == -1 ) cout << "Could not find clf_writer" << endl;
	  dirty = true;
	  break;
	case KEYCODE_X:
	  int k;
	  if((k = system("kill -9 `pidof clf_writer`")) == -1 ) cout << "Could not find clf_writer" << endl;
	  dirty = true;
	  break;
        }
    }
}



volatile bool laserWritten = false;

void writeLaser(ostream& os, const sensor_msgs::LaserScan::ConstPtr& scan){
  std::string frame_id = scan->header.frame_id;
  ros::Time t = scan->header.stamp;
  tf::StampedTransform laserPose, odomPose;
  std::string error;

  double rotThreshold = 0.05*0.05;
  double linThreshold = 0.1*0.1;

  if (listener->waitForTransform (odom_frame_id, laser_frame_id, t, ros::Duration(0.5), ros::Duration(0.01), &error)){
    listener->lookupTransform(odom_frame_id, laser_frame_id, t, laserPose);
    listener->lookupTransform(odom_frame_id, base_link_frame_id, t, odomPose);
    double odomX, odomY, odomTheta;
    double laserX, laserY, laserTheta;
    odomX=odomPose.getOrigin().x();
    odomY=odomPose.getOrigin().y();

    double yaw,pitch,roll;
    tf::Matrix3x3 mat =  odomPose.getBasis();
    mat.getEulerZYX(yaw, pitch, roll);
    odomTheta = yaw;
    
    uglyOdomX = odomX;
    uglyOdomY = odomY;
    uglyOdomTh = odomTheta;

    {
      double dx = lastOdomX - odomX;
      double dy = lastOdomY - odomY;
      double dth = lastOdomTh - odomTheta;
      if ( (dx*dx + dy*dy) < linThreshold  && dth*dth < rotThreshold)
	return;
    }
    
   
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
      if (scan->ranges[i]<scan->range_min || scan->ranges[i] > scan->range_max)
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
    cerr << ".";
    
    lastOdomX = odomX;
    lastOdomY = odomY;
    lastOdomTh = odomTheta;
    laserWritten = true;
  } else {
    std::cerr << error << std::endl; 
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  writeLaser(os, scan);
}

void laserCallbackScreen(const sensor_msgs::LaserScan::ConstPtr& scan) {
  writeLaser(cout, scan);
}
/*
void writeFiducial(std::ostream& os, const dis_stage::FiducialArray::ConstPtr& msg) {
  if (! laserWritten)
    return;

  for(unsigned int i = 0; i < msg->fiducialSet.size(); ++i)
  {
    dis_stage::Fiducial fiduciaReceived = msg->fiducialSet[i];
    os << "VERTEX_TAG " << fiduciaReceived.fiducial_id << " ";
    os << fiduciaReceived.position.x << " " << fiduciaReceived.position.y << " " << fiduciaReceived.position.z << " ";
    os << uglyOdomX << " " << uglyOdomY << " " << uglyOdomTh << " ";
    char timeString[50];
    sprintf(timeString, "%d.%08d", msg->header.stamp.sec, msg->header.stamp.nsec);
    os << timeString << " hostname ";
    os << timeString << endl;
    os << flush;
  }
//  os << "VERTEX_TAG " << msg->fiducial_id << " ";
 // os << msg->position.x << " " << msg->position.y << " " << msg->position.z << " ";
//  os << uglyOdomX << " " << uglyOdomY << " " << uglyOdomTh << " ";

//  char timeString[50];
//  sprintf(timeString, "%d.%08d", msg->header.stamp.sec, msg->header.stamp.nsec);
//  os << timeString << " hostname ";
//  os << timeString << endl;
//  os << flush;
  laserWritten = false;
}

void fiducialCallback(const dis_stage::FiducialArray::ConstPtr& msg)
{
  writeFiducial(os, msg);
}

void fiducialCallbackScreen(const dis_stage::FiducialArray::ConstPtr& msg)
{
  writeFiducial(cout, msg);
}
*/
void kinectCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  os << "VERTEX_TAG INVALID ";
  os << msg->point.x << " " << msg->point.y << " " << msg->point.z << " ";
  os << lastOdomX << " " << lastOdomY << " " << lastOdomTh << " ";

  char timeString[50];
  sprintf(timeString, "%d.%08d", msg->header.stamp.sec, msg->header.stamp.nsec);
  os << timeString << " hostname ";
  os << timeString << endl;
  os << flush;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "clf_writer");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  boost::thread t = boost::thread(&exitChar);

  np.getParam("odom_frame_id", odom_frame_id);
  np.getParam("base_link_frame_id", base_link_frame_id);
  np.getParam("laser_frame_id", laser_frame_id);
  np.getParam("laser_topic", laser_topic);
  np.getParam("fiducial_topic", fiducial_topic);
  np.getParam("carmen_filename", carmen_filename);
  cerr << "odom_frame_id " << odom_frame_id << endl;
  cerr << "base_link_frame_id " << base_link_frame_id << endl;
  cerr << "laser_frame_id " << laser_frame_id << endl;
  cerr << "laser_topic " << laser_topic << endl;
  cerr << "opening file \"" << carmen_filename << "\"" << endl;
  os.open(carmen_filename.c_str());
  listener = new tf::TransformListener;
    ros::Subscriber sub1 = n.subscribe(laser_topic, 100, laserCallback);
  //ros::Subscriber sub1a = n.subscribe(laser_topic, 100, laserCallbackScreen);
//    ros::Subscriber sub2 = n.subscribe(fiducial_topic, 100, fiducialCallback);
  //ros::Subscriber sub2a = n.subscribe(fiducial_topic, 100, fiducialCallbackScreen);
  ros::spin();

  t.interrupt();
  t.join();
  tcsetattr(kfd, TCSANOW, &cooked);

  return 0;
}
