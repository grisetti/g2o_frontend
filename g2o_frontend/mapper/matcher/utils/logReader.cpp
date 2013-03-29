#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

#include "logReader.h"

using namespace std;
using namespace Eigen;

LogReader::LogReader(const std::string& fileIn, int& res): _laserType(0), _startAngle(0.0), _fov(0.0), _angularResolution(0.0), _maxRange(0.0), _accuracy(0.0), _remissionMode(0), _readingsNumber(0), _remissionsNumber(0)
{
  _resolution = res;
  this->logToScan(fileIn);
}

LogReader::~LogReader() {;}

int LogReader::readLine(istream& is, stringstream& currentLine)
{
  if(is.eof())
  {
    return -1;
  }
  currentLine.str("");
  currentLine.clear();
  is.get(*currentLine.rdbuf());
  // fail is set on empty lines
  if(is.fail())
  {
    is.clear();
  }
  SKIP_LINE(is); // read \n not read by get()
  return currentLine.str().size();
}

bool LogReader::logToScan(const std::string& fileIn)
{
  cout << "Creating scans" << endl;
  ifstream ifs(fileIn.c_str(), ifstream::in);
  if(!ifs.good())
  {
    return false;
  }
  Vector3f previousPose;
  uint it = 0;
  
  while(!ifs.eof())
  {
    LaserData ld;
    Vector2fVector currentScan;
    int it = 0;
    stringstream currLine;
    int check = readLine(ifs, currLine);
    if(check == -1)
    {
      return false;
    }
    string label;
    currLine >> label;

    if(label == "ROBOTLASER1")
    {
      currLine >> _laserType >> _startAngle >> _fov >> _angularResolution >> _maxRange >> _accuracy >> _remissionMode >> _readingsNumber;
      ld.minAngle = _startAngle;
      ld.maxAngle = _fov - fabs(_startAngle);
      ld.angularRes = _angularResolution;
      ld.beams = _readingsNumber;

      for(size_t i=0; i < _readingsNumber; ++i)
      {
	const double alpha = _startAngle + i * _angularResolution;
	double radius;
	currLine >> radius;
	Vector2f p = polarToCartesian(alpha, radius);
	correspondence c;
// 	c.valid = 0;
// 	c.j1 = -1;
// 	c.j2 = -1;
// 	c.dist2_j1 = 10000;
	ld.points.push_back(p);
	ld.corr.push_back(c);
	if(radius < _maxRange)
	{
	  currentScan.push_back(p);
	  ld.mean += p;
	  ld.outOfRangeBeam.push_back(1);
	  it++;
	}
	else
	{
	  ld.outOfRangeBeam.push_back(0);
	}
      }
      ld.mean /= it;
      currLine >> _remissionsNumber;
      vector<double> remissions(_remissionsNumber);

      for(size_t i=0; i < _remissionsNumber; ++i)
      {
	currLine >> remissions[i];
      }

      float lx, ly, ltheta;
      currLine >> lx >> ly >> ltheta;
      Vector3f l(lx, ly, ltheta);
      ld.odometricLaserPose = l;
      float rx, ry, rtheta;
      currLine >> rx >> ry >> rtheta;
      Vector3f v(rx, ry, rtheta);
      ld.odometricRobotPose = v;

      if(it == 0)
      {
	scans.push_back(currentScan);
	poses.push_back(v);
	data.push_back(ld);
	previousPose = v;
      }
      else
      {
	float xD = fabs(v.x()-previousPose.x());
	float yD = fabs(v.y()-previousPose.y());
	float tD = fabs(v.z()-previousPose.z());
	//The robot should move a bit
	if((xD > 0.03) && (yD > 0.03) && ((tD > 0.03)))
	{
	  scans.push_back(currentScan);
	  poses.push_back(v);
	  data.push_back(ld);
	  previousPose = v;
	}
      }
    }    
    it++;
  }
  cout << scans.size() << " scans created" << endl;
  cout << data.size() << " elements created" << endl;
  ifs.close();

  return true;
}
