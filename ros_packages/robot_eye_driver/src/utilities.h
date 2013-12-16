#ifndef UTILITIES_H
#define UTILITIES_H

#include "Eigen/Core"
#include "RE05Driver.h"


double deg2rad(double deg){
  return deg*M_PI/180;
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

#endif // UTILITIES_H
