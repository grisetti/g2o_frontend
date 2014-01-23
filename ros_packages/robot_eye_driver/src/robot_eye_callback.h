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
    typedef std::vector<Eigen::Vector4f>  EuclideanMeasurements;
    typedef std::list<PolarMeasurements> PolarList;
    typedef std::list<EuclideanMeasurements> EuclideanList;


    inline double deg2rad(double deg){
        return deg*M_PI/180;
    }

    class LaserCB : public ocular::RobotEyeLaserDataCallbackClass{

    public:
        bool pop(PolarMeasurements& pm);
        bool pop(EuclideanMeasurements& em);
        Mutex& getMutex() {return _mutex_meas; }
        PolarList& measList() {return _pmlist; }
        EuclideanList& xyzMeasList() {return _emlist; }
        PolarMeasurements deg2radObservation(PolarMeasurements& pm);
        Eigen::Vector4f polar2euclidean(ocular::ocular_rbe_obs_t obs);

        void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations);

    protected:
        Mutex _mutex_meas;
        PolarList _pmlist;      // [az, el, range, intensity]
        EuclideanList _emlist;  // [x, y, z, intensity]
    };

    class NotificationCB : public ocular::RobotEyeNotificationCallbackClass{

        void NotificationCallback(ocular::ocular_error_t ErrCode){
            std::cerr << "notification callback triggered" << std::endl;
            if(ErrCode != 0)
                std::cerr << "with errorcode = " << ErrCode << std::endl;
        }
    };

    //class AngleCB : public ocular::RobotEyeApertureAnglesCallbackClass{

    //    void ApertureAnglesCallback(double az, double el, unsigned int timestamp){
    //        //        std::cerr << "angle callback triggered: az " << az << " ,el " << el << ", timestamp" << timestamp << std::endl;
    //        std::cerr << ".";
    //    }
    //};

}

#endif // ROBOT_EYE_CALLBACK_H
