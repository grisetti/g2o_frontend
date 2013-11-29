#ifndef _MY_CALLBACKS
#define _MY_CALLBACKS

#include <iostream>
#include </usr/local/RobotEye/include/roboteye/RobotEye.h>
#include </usr/local/RobotEye/include/roboteye/RE05Driver.h>

//#include "mutex.h"

std::vector<ocular::ocular_rbe_obs_t> measurements;
//Mutex* mutex_meas;


class AngleCB : public ocular::RobotEyeApertureAnglesCallbackClass{

  virtual void ApertureAnglesCallback(double az, double el, unsigned int timestamp){
        std::cerr << "angle callback triggered: az " << az << " ,el " << el << ", timestamp" << timestamp << std::endl;
  }
};

class LaserCB : public ocular::RobotEyeLaserDataCallbackClass{

  virtual void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations){
    std::cerr << "laser callback triggered" << std::endl;
    // in case of parallel callbacks that would modify the measurements vector, not needed
//    mutex_meas->lock();
    for(int i = 0; i<= observations.size(); i++){
        measurements.push_back(observations[i]);
    }
//    mutex_meas->unlock();
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


