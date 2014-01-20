#include "robot_eye_callback.h"

namespace roboteye {

    LaserCB::LaserCB(){

//        _mutex_meas = 0;
    }

    bool LaserCB::pop(PolarMeasurements& m){

         if(_pmlist.empty())
             return false;

        _mutex_meas.lock();
        m = _pmlist.front();
        _pmlist.pop_front();
        _mutex_meas.unlock();

        return true;
    }

    void LaserCB::LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations){
        //    std::cerr << "laser callback triggered" << std::endl;
        std::cerr /*<< observations.size()*/ << " . ";

        _mutex_meas.lock();

        //list to be published
        _pmlist.push_back(observations);

        /// just to print the data
//        _meas_current.clear();
//        _xyz_meas_current.clear();

        roboteye::EuclideanMeasurements xyz_meas_tmp;
        for(unsigned int i = 0; i <= observations.size(); i++){
            Eigen::Vector4f xyzi = polar2euclidean(observations[i]);
            xyz_meas_tmp.push_back(xyzi);
//            _meas_current.push_back(observations[i]);
//            _xyz_meas_current.push_back(xyzi);
        }
//        _meas_all_vector.push_back(observations);
        _xyz_meas_all_vector.push_back(xyz_meas_tmp);


        _mutex_meas.unlock();
    }

    Eigen::Vector4f LaserCB::polar2euclidean(ocular::ocular_rbe_obs_t obs){

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

}
