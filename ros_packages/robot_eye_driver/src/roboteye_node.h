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

namespace roboteye
{
    class roboteye_node
    {
    public:
        roboteye_node(double az_rate, double n_lines, double laser_freq, double averaging, bool intensity, std::string outfilename);
        ~roboteye_node();

        ros::Publisher scanPub() {return _scan_pub; }
        RobotEyeScan scan() {return _scan; }
        void setScan(RobotEyeScan &scan);
        RobotEyeConfig config() {return _scan_config; }
        void setConfig(RobotEyeConfig &scan_config);
        LaserCB laserCallBack() {return _laser_callback; }
        void setCallback(LaserCB laserCallback) {_laser_callback = laserCallback; }
        unsigned int numReadings() {return _num_readings; }
        void setNumReadings(unsigned int nr) {_num_readings = nr; }
        ros::Time lastStamp() {return _lastStamp; }
        void setLastStamp(ros::Time ls) {_lastStamp = ls; }

        void stopAndPrint();
        void printAndWriteLaserData(std::string outfilename);
        void roboteyeRunning();

        // config parameters
        std::string _sensor_IP;
        double _az_rate;
        double _N_lines;
        double _laser_freq;
        double _averaging;
        bool _intensity;
        std::string _outfilename;
        //    int _seconds;

    protected:
        /*Ros specific stuff*/
        ros::NodeHandle _handle;
        ros::Publisher _scan_pub;
        roboteye::RobotEyeConfig _scan_config;
        roboteye::RobotEyeScan _scan;
        ros::Time _lastStamp;
        //    std::vector<roboteye::RobotEyeScan<> _scan_vector;

        LaserCB _laser_callback;
        ocular::RE05Driver* laserone;
        //    AngleCB _angle_callback;
        //    NotificationCB _notif_callback;
        unsigned int _num_readings;
        //    double _desired_freq;

        /*Boost specific stuff*/
        boost::thread _thrd;
    //    boost::mutex mtx;

    };

}

#endif // ROBOTEYE_NODE_H
