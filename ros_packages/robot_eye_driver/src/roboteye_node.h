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

#include "roboteyestruct.h"
#include "robot_eye_callback.h"

#include <ros/ros.h>
#include <robot_eye_driver/RobotEyeScan.h>

class roboteye_node
{
public:
    roboteye_node(double az_rate, double n_lines, double laser_freq, double averaging, bool intensity);
    ~roboteye_node();

    roboteye::RobotEyeConfig getConfig() {return _scan_config; }
    void setConfig(roboteye::RobotEyeConfig scan_config) {_scan_config = scan_config; }
    void stopAndPrint();
    void printAndWriteLaserData();
    void roboteyeRunning();

    // config parameters
    std::string _sensor_IP;
//    int _seconds;
    double _az_rate;
    double _N_lines;
    double _laser_freq;
    double _averaging;
    bool _intensity;


protected:
//    AngleCB _angle_callback;
    LaserCB _laser_callback;
//    NotificationCB _notif_callback;
    ocular::RE05Driver* laserone;
    unsigned int _num_readings;
    double _desired_freq;
    double _lastStamp;

    /*Ros specific stuff*/
    ros::NodeHandle _handle;
    ros::Publisher _scan_pub;
    robot_eye_driver::RobotEyeScan _scan_msg;
    roboteye::RobotEyeConfig _scan_config;
    //    std::vector<roboteye::RobotEyeScan> _scan_vector;
    roboteye::RobotEyeScan _scan;

    /*Boost specific stuff*/
    boost::thread _thrd;
//    boost::mutex mtx;

};


#endif // ROBOTEYE_NODE_H
