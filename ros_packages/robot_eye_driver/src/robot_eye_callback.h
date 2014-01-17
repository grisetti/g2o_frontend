#ifndef ROBOT_EYE_CALLBACK_H
#define ROBOT_EYE_CALLBACK_H

#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <Eigen/Core>
//#include <boost/thread/thread.hpp>

#include "RE05Driver.h"
//#include "mutex.h"

typedef std::vector<ocular::ocular_rbe_obs_t>  PolarMeasurements;
typedef std::vector<PolarMeasurements> PolarVector;

typedef std::vector<Eigen::Vector4f>  EuclideanMeasurements;
typedef std::vector<EuclideanMeasurements> EuclideanVector;


inline double deg2rad(double deg){
  return deg*M_PI/180;
}


class LaserCB : public ocular::RobotEyeLaserDataCallbackClass{

public:
//    Mutex _mutex_meas;
    PolarMeasurements _meas_all; // [az, el, range, intensity]
    PolarMeasurements _meas_current; // [az, el, range, intensity]
    PolarVector _meas_all_vector;
    EuclideanMeasurements _xyz_meas_all; // [x, y, z, intensity]
    EuclideanMeasurements _xyz_meas_current; // [x, y, z, intensity]
    EuclideanVector _xyz_meas_all_vector;

    virtual void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations){
        //    std::cerr << "laser callback triggered" << std::endl;
        std::cerr << observations.size() << " ";

        // in case of parallel callbacks that would modify the measurements vector, not needed
//        _mutex_meas.lock();

        _meas_current.clear();
        _xyz_meas_current.clear();

        EuclideanMeasurements xyz_meas_tmp;

        for(unsigned int i = 0; i <= observations.size(); i++){
            Eigen::Vector4f xyzi = polar2euclidean(observations[i]);
            xyz_meas_tmp.push_back(xyzi);

            _meas_current.push_back(observations[i]);
            _xyz_meas_current.push_back(xyzi);

            //adding the current measurement in the global structures to cout/print the whole data at the end
            _meas_all.push_back(observations[i]);
//            _xyz_meas_all.push_back(xyzi);

        }
        _meas_all_vector.push_back(observations);
        _xyz_meas_all_vector.push_back(xyz_meas_tmp);

//        _mutex_meas.unlock();
    }

    Eigen::Vector4f polar2euclidean(ocular::ocular_rbe_obs_t obs){

      // az : rotation around z-axis (on this sensor it's considered positive in clockwise way)
      // el : rotation around y-axis (a negative rotation means to look down (positive in counterclockwise way))

      float az = deg2rad(-obs.azimuth);
      float el = deg2rad(-obs.elevation);
      float t = obs.range;
      unsigned short int intensity = obs.intensity;


      Eigen::Matrix4f rot_az, rot_el;
      Eigen::Vector4f trasl(t, 0, 0, 1);
      rot_az <<
        cos(az), -sin(az), 0, 0,
        sin(az),  cos(az), 0, 0,
        0      ,   0     , 1, 0,
        0      ,   0     , 0, 1;

      rot_el <<
        cos(el) ,  0 , sin(el), 0,
        0       ,  0 ,  0     , 0,
        -sin(el),  0 , cos(el), 0,
        0       ,  0 ,  0     , 1;

      //computing the xyz_intensity coordinate in the Euclidean Space
      Eigen::Vector4f xyz_intensity = rot_az * rot_el * trasl;
      xyz_intensity[3] = intensity;

      return xyz_intensity;
    }

};

class AngleCB : public ocular::RobotEyeApertureAnglesCallbackClass{

    virtual void ApertureAnglesCallback(double az, double el, unsigned int timestamp){
        //        std::cerr << "angle callback triggered: az " << az << " ,el " << el << ", timestamp" << timestamp << std::endl;
        std::cerr << ".";
    }
};
class NotificationCB : public ocular::RobotEyeNotificationCallbackClass{

    virtual void NotificationCallback(ocular::ocular_error_t ErrCode){
        std::cerr << "notification callback triggered" << std::endl;
        if(ErrCode != 0)
            std::cerr << "with errorcode = " << ErrCode << std::endl;
    }
};

#endif // ROBOT_EYE_CALLBACK_H
