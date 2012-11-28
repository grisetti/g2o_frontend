/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#ifndef LASERROBOTDATA_H
#define LASERROBOTDATA_H


#include <iosfwd>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "g2o/core/hyper_graph.h"
#include "g2o_frontend/thesis/SensorData.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "SensorLaserRobot.h"

/**
* \brief laser measurement obtained by a robot
*
* A laser measurement obtained by a robot. The measurement is equipped with a pose of the robot at which
* the measurement was taken. The read/write function correspond to the CARMEN logfile format.
*/
class LaserRobotData: public SensorData {
public:
	
	typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;
	
	LaserRobotData();
	virtual ~LaserRobotData();
	//! read the data from a stream
	virtual bool read(std::istream& is);
	//! write the data to a stream
	virtual bool write(std::ostream& os) const;	
  //virtual void writeOut();
	//void update();
	//void release();
  inline int paramIndex() {return _paramIndex;}	
	const std::string& baseFilename() const { return _baseFilename; };
	void  setBaseFilename(const std::string baseFilename_) { _baseFilename = baseFilename_; };
	virtual Sensor* getSensor() const { return _laserRobotSensor; }
	virtual void setSensor(Sensor* laserRobotSensor_);
	
	//! odom of the robot
  const g2o::SE2& odomPose() const { return _odomPose;}
  void setOdomPose(const g2o::SE2& odomPose);
	//! position of the laser wrt the world
	g2o::SE2 laserPose() const { return _odomPose * _laserRobotSensor->laserParams().laserPose; };
	
	double timestamp() const { return _timestamp;}
	void setTimestamp(double ts);

	double loggerTimestamp() const { return _loggerTimestamp;}
	void setLoggerTimestamp(double ts);

// 	const std::string& tag() const { return _tag;}
// 	void setTag(const std::string& tag);

	const std::string& hostname() const { return _hostname;}
	void setHostname(const std::string& hostname);
	/**
	* computes a cartesian view of the beams (x,y).
	* @return a vector with the points of the scan in cartesian coordinates.
	*/
	Point2DVector cartesian() const;

protected:
	std::string _baseFilename;
  SensorLaserRobot* _laserRobotSensor;
	g2o::SE2 _odomPose;
	double _timestamp; ///< timestamp when the measurement was generated
  double _loggerTimestamp; ///< timestamp when the measurement was recorded
	//std::string _tag; ///< string tag (FLASER, ROBOTLASER, ODOM..) of the line in the log
  std::string _hostname; ///< name of the computer/robot generating the data
	//! velocities and safety distances of the robot
	double _laserTv, _laserRv, _forwardSafetyDist, _sideSaftyDist, _turnAxis;
	
private:
  int _paramIndex;
};	

#ifdef G2O_HAVE_OPENGL

class LaserRobotDataDrawAction : public g2o::DrawAction{
public:
  LaserRobotDataDrawAction() : DrawAction(typeid(LaserRobotData).name()) {};
  virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element, 
					      																g2o::HyperGraphElementAction::Parameters* params_ );
protected:
  virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
  g2o::IntProperty* _beamsDownsampling;
  g2o::FloatProperty* _pointSize;
  g2o::FloatProperty* _maxRange;
};

#endif

#endif // LASERROBOTDATA_H
