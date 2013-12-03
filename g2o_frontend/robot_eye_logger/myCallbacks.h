#ifndef _MY_CALLBACKS
#define _MY_CALLBACKS

#include <iostream>
#include "Eigen/Core"
#include "utilities.h"

#include "mutex.h"

std::vector<ocular::ocular_rbe_obs_t> _measurements; // [az, el, range, intensity]
std::vector<Eigen::Vector4d> _xyz_meas; // [x, y, z, intensity]
Mutex _mutex_meas;


class AngleCB : public ocular::RobotEyeApertureAnglesCallbackClass{

  virtual void ApertureAnglesCallback(double az, double el, unsigned int timestamp){
//        std::cerr << "angle callback triggered: az " << az << " ,el " << el << ", timestamp" << timestamp << std::endl;
        std::cerr << ".";
  }
};

class LaserCB : public ocular::RobotEyeLaserDataCallbackClass{

  virtual void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations){
//    std::cerr << "laser callback triggered" << std::endl;
    std::cerr << ".";
    // in case of parallel callbacks that would modify the measurements vector, not needed
    _mutex_meas.lock();
    for(unsigned int i = 0; i <= observations.size(); i++){
        Eigen::Vector4d xyz_int = polar2euclidean(observations[i]);
        _xyz_meas.push_back(xyz_int);
        _measurements.push_back(observations[i]); //to print the data
    }
    _mutex_meas.unlock();
  }
};

class NotificationCB : public ocular::RobotEyeNotificationCallbackClass{

    virtual void NotificationCallback(ocular::ocular_error_t ErrCode){
        std::cerr << "notification callback triggered" << std::endl;
        if(ErrCode != 0)
          std::cerr << "with errorcode = " << ErrCode << std::endl;
    }
};

#endif //_MY_CALLBACKS


