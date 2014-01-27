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
        roboteye_node();
        roboteye_node(double az_rate, double n_lines, double laser_freq, double averaging, bool intensity, std::string outfilename);
        ~roboteye_node();

        double laserFreq() {return _laser_freq; }
        void setLaserFreq(double lf) {_laser_freq = lf; }
        ros::Publisher scanPub() {return _scan_pub; }
        RobotEyeScan scan() {return _scan; }
        void setScan(RobotEyeScan &scan);
        RobotEyeConfig config() {return _scan_config; }
        void setConfig(RobotEyeConfig &scan_config);
        LaserCB& laserCallBack() {return _laser_callback; }
        void setCallback(LaserCB laserCallback) {_laser_callback = laserCallback; }
        ros::Time lastStamp() {return _lastStamp; }
        void setLastStamp(ros::Time ls) {_lastStamp = ls; }
        unsigned int numReadings() {return _num_readings; }
        void setNumReadings(unsigned int nr) {_num_readings = nr; }
        bool isRunning() {return _isrunning;}
        void setIsRunning();
        roboteyeState state() {return _state;}
        void setState(roboteyeState s) {_state = s;}

        void stop();
        void setDesideredApertureAngles();
        void printAndWriteLaserData(std::string outfilename);
        void roboteyeRun();
        void roboteyeStop();
        void roboteyePause();

    protected:
        // config parameters
        std::string _sensor_IP;
        double _az_rate;
        double _N_lines;
        double _laser_freq;
        double _averaging;
        bool _intensity;
        std::string _outfilename;
        //    int _seconds;

        /*Ros specific stuff*/
        ros::NodeHandle _handle;
        ros::Publisher _scan_pub;
        roboteye::RobotEyeConfig _scan_config;
        roboteye::RobotEyeScan _scan;
        ros::Time _lastStamp;

        /*Laser Callback specific stuff*/
        LaserCB _laser_callback;
        ocular::RE05Driver* laserone;
        NotificationCB _notif_callback;
        //    AngleCB _angle_callback;
        unsigned int _num_readings;

        roboteyeState _state;
        bool _isrunning;

        /*Boost specific stuff*/
        boost::thread _thrd;
        //    boost::mutex mtx;

    };

}

#endif // ROBOTEYE_NODE_H
