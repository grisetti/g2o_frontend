#ifndef ROBOTEYESTRUCT_H
#define ROBOTEYESTRUCT_H

#include <iostream>
#include <stdint.h>
#include <Eigen/Core>

namespace roboteye
{
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
//        //! Range Resolution [m]
//        float range_res;
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

    enum roboteyeState
    {
       STOP,
       RUN,
       PAUSE
    };
}

#endif // ROBOTEYESTRUCT_H
