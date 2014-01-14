#ifndef ROBOTEYE_H
#define ROBOTEYE_H

#include <iostream>
#include <stdint.h>
#include "Eigen/Core"

#include "utilities.h"

#include "mutex.h"

namespace roboteye
{
    typedef std::vector<ocular::ocular_rbe_obs_t>  PolarMeasurements;
    typedef std::vector<PolarMeasurements> PolarVector;

    typedef std::vector<Eigen::Vector4d>  EuclideanMeasurements;
    typedef std::vector<EuclideanMeasurements> EuclideanVector;


    //global variables
    PolarMeasurements _meas_all; // [az, el, range, intensity]
    PolarVector _meas_all_vector;
    EuclideanMeasurements _xyz_meas_all; // [x, y, z, intensity]
    EuclideanVector _xyz_meas_all_vector;

    Mutex _mutex_meas;

    //! A struct for returning configuration from the RobotEye
    struct RobotEyeConfig
    {
        //! Start orizontal angle for the laser scan [deg].  0 is forward and angles are considered positive in clockwise way.
        float min_azimuth;
        //! Stop orizontal angle for the laser scan [deg].   0 is forward and angles are considered positive in clockwise way.
        float max_azimuth;
        //! Scan orizontal resolution [deg].
        float azim_increment;
        //! Start vertical angle for the laser scan [deg].  a positive rotation means to look up (angles measured counterclockwise way)
        float min_elevation;
        //! Stop vertical angle for the laser scan [deg].   a positive rotation means to look up (angles measured counterclockwise way)
        float max_elevation;
        //! Scan vertical resolution [deg].
        float elev_increment;
        //! Scan resoltuion [s]
        float time_increment;
        //! Time between scans
        float scan_time;
        //! Minimum range [m]
        float min_range;
        //! Maximum range [m]
        float max_range;
        //! Range Resolution [m]
        float range_res;
    };


    //! A struct for returning laser readings from the RobotEye
    struct RobotEyeScan
    {
        //! Array of ranges
        std::vector<Eigen::Vector3d> ranges;
        //! Array of intensities
        std::vector<float> intensities;
        //! Self reported time stamp in nanoseconds
        uint64_t self_time_stamp;
        //! System time when first range was measured in nanoseconds
        uint64_t system_time_stamp;
        //! Configuration of scan
        RobotEyeConfig config;
    };


    class AngleCB : public ocular::RobotEyeApertureAnglesCallbackClass{

        virtual void ApertureAnglesCallback(double az, double el, unsigned int timestamp){
            //        std::cerr << "angle callback triggered: az " << az << " ,el " << el << ", timestamp" << timestamp << std::endl;
            std::cerr << ".";
        }
    };

    class LaserCB : public ocular::RobotEyeLaserDataCallbackClass{

        virtual void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations){
            //    std::cerr << "laser callback triggered" << std::endl;
            std::cerr << observations.size() << " ";
            // in case of parallel callbacks that would modify the measurements vector, not needed
            //    _mutex_meas.lock();

            _meas_all_vector.push_back(observations);
            EuclideanMeasurements xyz_meas_tmp;

            for(unsigned int i = 0; i <= observations.size(); i++){
                Eigen::Vector4d xyzi = polar2euclidean(observations[i]);
                xyz_meas_tmp.push_back(xyzi);

                //adding the current measurement in the global structures
                _meas_all.push_back(observations[i]); //to cout/plot the data
                _xyz_meas_all.push_back(xyzi);
            }
            _xyz_meas_all_vector.push_back(xyz_meas_tmp);

            //    _mutex_meas.unlock();
        }
    };

    class NotificationCB : public ocular::RobotEyeNotificationCallbackClass{

        virtual void NotificationCallback(ocular::ocular_error_t ErrCode){
            std::cerr << "notification callback triggered" << std::endl;
            if(ErrCode != 0)
                std::cerr << "with errorcode = " << ErrCode << std::endl;
        }
    };

}
#endif // ROBOTEYE_H
