#include "robot_eye_callback.h"

namespace roboteye {

    bool LaserCB::pop(PolarMeasurements& pm){

         if(_pmlist.empty())
             return false;

        _mutex_meas.lock();
        pm = _pmlist.front();
        _pmlist.pop_front();
        _mutex_meas.unlock();

        return true;
    }
    bool LaserCB::pop(EuclideanMeasurements& em){

         if(_emlist.empty())
             return false;

        _mutex_meas.lock();
        em = _emlist.front();
        _emlist.pop_front();
        _mutex_meas.unlock();

        return true;
    }

    void LaserCB::LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations){
        //    std::cerr << "laser callback triggered" << std::endl;
        std::cerr /*<< observations.size()*/<< " . ";

        _mutex_meas.lock();

        PolarMeasurements radObservations = deg2radObservation(observations);
        //list to be published
        _pmlist.push_back(radObservations);

        // conversion to euclidean to create a pcd cloud
        roboteye::EuclideanMeasurements xyz_meas;
        for(unsigned int i = 0; i < observations.size(); i++){
            Eigen::Vector4f xyzi = polar2euclidean(observations[i]);
            xyz_meas.push_back(xyzi);
        }
        _emlist.push_back(xyz_meas);

        _mutex_meas.unlock();
    }

    PolarMeasurements LaserCB::deg2radObservation(PolarMeasurements &pm) {

        PolarMeasurements radObservations;
        ocular::ocular_rbe_obs_t tmp;
        for(unsigned int i = 0; i <= pm.size(); i++){
            tmp = pm[i];
            float az = deg2rad(-tmp.azimuth);
            float el = deg2rad(-tmp.elevation);
            tmp.azimuth = az;
            tmp.elevation = el;
//            std::cerr << "azim: " << tmp.azimuth << ",\telev: " << tmp.elevation << ",\trange: " << tmp.range << ",\tintensity: " << tmp.intensity << std::endl;
            radObservations.push_back(tmp);
        }
        return radObservations;
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
