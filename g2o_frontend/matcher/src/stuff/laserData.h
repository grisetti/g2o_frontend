#ifndef __LASER_DATA_H__
#define __LASER_DATA_H__

#include <eigen3/Eigen/Core>
#include <vector>

typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

struct correspondence
{
  int valid; 
  int j1;
  int j2;
  double dist2_j1;
  
  correspondence()
  {
    valid = 0;
    j1 = -1;
    j2 = -1;
    dist2_j1 = 10000;
  }
};

class LaserData
{
public:
  LaserData();
  ~LaserData();
  
  int beams;
  float angularRes;
  float minAngle;
  float maxAngle;
  int validBeams;
  
  Eigen::Vector2f mean;
  Eigen::Vector3f odometricLaserPose;  
  Eigen::Vector3f odometricRobotPose;  
  
  std::vector<int> outOfRangeBeam;
  Vector2fVector points;
  Vector2fVector modifiedPoints;
  std::vector<correspondence> corr;
};

#endif