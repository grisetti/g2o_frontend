#ifndef __LOG_READER_H__
#define __LOG_READER_H__

#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "laserData.h"

#define SKIP_LINE(f) \
   {char c=' ';while(c != '\n' && f.good() && !(f).eof()) (f).get(c);}

typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

class LogReader
{
public:
  
  LogReader(const std::string&, int&);
  ~LogReader();

  inline const std::vector<LaserData> getData() const {return data;}
  inline const std::vector<Vector2fVector> getScans() const {return scans;}
  inline const std::vector<Eigen::Vector3f> getPoses() const {return poses;}
  
  bool logToScan(const std::string&);
  
private:
  inline const Eigen::Vector2f polarToCartesian(const double& alpha, const double& r) const
  {
    Eigen::Vector2f p(cos(alpha)*r, sin(alpha)*r);
    return p;
  }

  int readLine(std::istream&, std::stringstream&);
  
  std::vector<Vector2fVector> scans;
  std::vector<LaserData> data;
  std::vector<Eigen::Vector3f> poses;
  
  float _resolution;
  int _laserType;
  double _startAngle, _fov, _angularResolution, _maxRange, _accuracy;
  int _remissionMode, _readingsNumber, _remissionsNumber;
};
#endif
