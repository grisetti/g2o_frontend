#ifndef ROBOT_EYE_CALLBACK_H
#define ROBOT_EYE_CALLBACK_H

#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <Eigen/Core>
//#include <boost/thread/thread.hpp>

#include <list>
#include "RE05Driver.h"
#include "mutex.h"
#include "roboteyestruct.h"

namespace roboteye
{
    typedef std::vector<ocular::ocular_rbe_obs_t>  PolarMeasurements;
    typedef std::vector<PolarMeasurements> PolarVector;

    typedef std::vector<Eigen::Vector4f>  EuclideanMeasurements;
    typedef std::vector<EuclideanMeasurements> EuclideanVector;


    inline double deg2rad(double deg){
        return deg*M_PI/180;
    }

    class roboteye_node;

    class LaserCB : public ocular::RobotEyeLaserDataCallbackClass{

    public:

        LaserCB();
        bool pop(PolarMeasurements& p);
        Mutex & getMutex() {return _mutex_meas; }
//        PolarVector& measAllVector() {return _meas_all_vector; }
        EuclideanVector& xyzMeasAllVector() {return _xyz_meas_all_vector; }

        virtual void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations);
        Eigen::Vector4f polar2euclidean(ocular::ocular_rbe_obs_t obs);

    protected:
        Mutex _mutex_meas;
        std::list<PolarMeasurements> _pmlist;

//        PolarMeasurements _meas_current; // [az, el, range, intensity]
//        PolarVector _meas_all_vector;
//        EuclideanMeasurements _xyz_meas_current; // [x, y, z, intensity]
        EuclideanVector _xyz_meas_all_vector;
    };

    //class AngleCB : public ocular::RobotEyeApertureAnglesCallbackClass{

    //    virtual void ApertureAnglesCallback(double az, double el, unsigned int timestamp){
    //        //        std::cerr << "angle callback triggered: az " << az << " ,el " << el << ", timestamp" << timestamp << std::endl;
    //        std::cerr << ".";
    //    }
    //};
    //class NotificationCB : public ocular::RobotEyeNotificationCallbackClass{

    //    virtual void NotificationCallback(ocular::ocular_error_t ErrCode){
    //        std::cerr << "notification callback triggered" << std::endl;
    //        if(ErrCode != 0)
    //            std::cerr << "with errorcode = " << ErrCode << std::endl;
    //    }
    //};

}

#endif // ROBOT_EYE_CALLBACK_H
