#ifndef ROBOTEYE_NODE_H
#define ROBOTEYE_NODE_H

#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <g2o/stuff/command_args.h>

//#include "roboteye.h"

#include <ros/ros.h>
#include <robot_eye_driver/RobotEyeScan.h>

//// parameters
//std::string outfilename;
//std::string _sensor_IP = "169.254.111.100";
//int _seconds;
//double _az_rate;
//double _N_lines;
//double _laser_freq;
//double _averaging;
//bool _intensity;


//// global variables
//roboteye::AngleCB _angle_callback;
//roboteye::LaserCB _laser_callback;
//roboteye::NotificationCB _notif_callback;
//ocular::RE05Driver* laserone;


class roboteye_node
{
public:
    roboteye_node();
};

#endif // ROBOTEYE_NODE_H
