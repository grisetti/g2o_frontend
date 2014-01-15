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

#include "RE05Driver.h"
#include "roboteyestruct.h"
//#include "mutex.h"

#include <ros/ros.h>
#include <robot_eye_driver/RobotEyeScan.h>


////global variables
//roboteye::PolarMeasurements _meas_all; // [az, el, range, intensity]
//roboteye::PolarMeasurements _meas_current; // [az, el, range, intensity]
//roboteye::PolarVector _meas_all_vector;
//roboteye::EuclideanMeasurements _xyz_meas_all; // [x, y, z, intensity]
//roboteye::EuclideanMeasurements _xyz_meas_current; // [x, y, z, intensity]
//roboteye::EuclideanVector _xyz_meas_all_vector;

typedef std::vector<ocular::ocular_rbe_obs_t>  PolarMeasurements;
typedef std::vector<PolarMeasurements> PolarVector;

typedef std::vector<Eigen::Vector4d>  EuclideanMeasurements;
typedef std::vector<EuclideanMeasurements> EuclideanVector;

inline double deg2rad(double deg){
  return deg*M_PI/180;
}

class AngleCB : public ocular::RobotEyeApertureAnglesCallbackClass{

    virtual void ApertureAnglesCallback(double az, double el, unsigned int timestamp){
        //        std::cerr << "angle callback triggered: az " << az << " ,el " << el << ", timestamp" << timestamp << std::endl;
        std::cerr << ".";
    }
};

class LaserCB : public ocular::RobotEyeLaserDataCallbackClass{

protected:
    PolarMeasurements _meas_all; // [az, el, range, intensity]
    PolarMeasurements _meas_current; // [az, el, range, intensity]
    PolarVector _meas_all_vector;
    EuclideanMeasurements _xyz_meas_all; // [x, y, z, intensity]
    EuclideanMeasurements _xyz_meas_current; // [x, y, z, intensity]
    EuclideanVector _xyz_meas_all_vector;
    //    Mutex _mutex_meas;

public:
    virtual void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations){
        //    std::cerr << "laser callback triggered" << std::endl;
        std::cerr << observations.size() << " ";

        // in case of parallel callbacks that would modify the measurements vector, not needed
//            _mutex_meas.lock();

        _meas_current.clear();
        _xyz_meas_current.clear();

        EuclideanMeasurements xyz_meas_tmp;

        for(unsigned int i = 0; i <= observations.size(); i++){
            Eigen::Vector4d xyzi = polar2euclidean(observations[i]);
            xyz_meas_tmp.push_back(xyzi);

            _meas_current.push_back(observations[i]);
            _xyz_meas_current.push_back(xyzi);

            //adding the current measurement in the global structures to cout/print the whole data at the end
            _meas_all.push_back(observations[i]);
            _xyz_meas_all.push_back(xyzi);

        }
        _meas_all_vector.push_back(observations);
        _xyz_meas_all_vector.push_back(xyz_meas_tmp);

//            _mutex_meas.unlock();
    }
    Eigen::Vector4d polar2euclidean(ocular::ocular_rbe_obs_t obs){

      // az : rotation around z-axis (on this sensor it's considered positive in clockwise way)
      // el : rotation around y-axis (a negative rotation means to look down (positive in counterclockwise way))

      double az = deg2rad(-obs.azimuth);
      double el = deg2rad(-obs.elevation);
      double t = obs.range;
      unsigned short int intensity = obs.intensity;

      double c_az = cos(az);
      double s_az = sin(az);

      double c_el = cos(el);
      double s_el = sin(el);


      Eigen::Matrix4d rot_az, rot_el;
      Eigen::Vector4d trasl(t, 0, 0, 1);
      rot_az <<
        c_az, -s_az, 0, 0,
        s_az,  c_az, 0, 0,
        0   ,   0  , 1, 0,
        0   ,   0  , 0, 1;

      rot_el <<
        c_el ,  0 , s_el, 0,
        0    ,  0 ,  0  , 0,
        -s_el,  0 , c_el, 0,
        0    ,  0 ,  0  , 1;

      //computing the xyz_intensity coordinate in the Euclidean Space
      Eigen::Vector4d xyz_intensity = rot_az * rot_el * trasl;
      xyz_intensity[3] = intensity;

      return xyz_intensity;
    }

};

class NotificationCB : public ocular::RobotEyeNotificationCallbackClass{

    virtual void NotificationCallback(ocular::ocular_error_t ErrCode){
        std::cerr << "notification callback triggered" << std::endl;
        if(ErrCode != 0)
            std::cerr << "with errorcode = " << ErrCode << std::endl;
    }
};

class roboteye_node
{
public:
    roboteye_node();
    // config parameters
    std::string _sensor_IP;


protected:
    AngleCB _angle_callback;
    LaserCB _laser_callback;
    NotificationCB _notif_callback;
    ocular::RE05Driver* laserone;
};


#endif // ROBOTEYE_NODE_H
