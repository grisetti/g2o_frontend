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


#include "LaserRobotData.h"
#include <fstream>
#include "g2o/stuff/macros.h"
#include "g2o/core/factory.h"
#include <Eigen/Dense>
#include <iomanip>
#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

using namespace g2o;
using namespace std;

LaserRobotData::LaserRobotData(): 
		_laserTv(0.), _laserRv(0.), _forwardSafetyDist(0.), _sideSaftyDist(0.), _turnAxis(0.)
{
	_paramIndex = -1;
	_baseFilename = "none";
	_laserRobotSensor = 0;
	
}

LaserRobotData::~LaserRobotData()
{
}

bool LaserRobotData::read(istream& is)
{
	int type;
	double angle, fov, res, maxrange, acc;
	int remission_mode;
	is >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;

	int beams;
	is >> beams;
	_laserRobotSensor->_laserParams = LaserParameters(type, beams, angle, res, maxrange, acc, remission_mode);
	_laserRobotSensor->_ranges.resize(beams);
	for (int i=0; i<beams; i++)
		is >> _laserRobotSensor->_ranges[i];

	is >> beams;
	_laserRobotSensor->_remissions.resize(beams);
	for (int i = 0; i < beams; i++)
		is >> _laserRobotSensor->_remissions[i];

	// special robot laser stuff
	double x,y,theta;
	is >> x >> y >> theta;
	SE2 lp(x,y,theta);
	//cerr << "x: " << x << " y:" << y << " th:" << theta << " ";
	is >> x >> y >> theta;
	//cerr << "x: " << x << " y:" << y << " th:" << theta;
	_odomPose = SE2(x,y,theta);
	_laserRobotSensor->_laserParams.laserPose = _odomPose.inverse()*lp;
	is >> _laserTv >>  _laserRv >>  _forwardSafetyDist >> _sideSaftyDist >> _turnAxis;

	// timestamp + host
	is >> _timestamp;
	is >> _hostname;
	is >> _loggerTimestamp;
	return true;
}

bool LaserRobotData::write(ostream& os) const
{
		os << _laserRobotSensor->_laserParams.type << " " << _laserRobotSensor->_laserParams.firstBeamAngle << " " << _laserRobotSensor->_laserParams.fov << " "
      << _laserRobotSensor->_laserParams.angularStep << " " << _laserRobotSensor->_laserParams.maxRange << " " << _laserRobotSensor->_laserParams.accuracy << " "
      << _laserRobotSensor->_laserParams.remissionMode << " ";
    os << _laserRobotSensor->ranges().size();
    for (size_t i = 0; i < _laserRobotSensor->ranges().size(); ++i)
      os << " " << _laserRobotSensor->ranges()[i];
    os << " " << _laserRobotSensor->_remissions.size();
    for (size_t i = 0; i < _laserRobotSensor->_remissions.size(); ++i)
      os << " " << _laserRobotSensor->_remissions[i];

    // odometry pose
    Eigen::Vector3d p = (_odomPose * _laserRobotSensor->_laserParams.laserPose).toVector();
    os << " " << p.x() << " " << p.y() << " " << p.z();
    p = _odomPose.toVector();
    os << " " << p.x() << " " << p.y() << " " << p.z();

    // crap values
    os << FIXED(" " <<  _laserTv << " " <<  _laserRv << " " << _forwardSafetyDist << " "
        << _sideSaftyDist << " " << _turnAxis);
    os << FIXED(" " << timestamp() << " " << hostname() << " " << loggerTimestamp());

    return os.good();
}

void LaserRobotData::setOdomPose(const g2o::SE2& odomPose_)
{
	_odomPose = odomPose_;
}


void LaserRobotData::setSensor(Sensor* laserRobotsensor_)
{
	_laserRobotSensor = dynamic_cast<SensorLaserRobot*>(laserRobotsensor_);
}

void LaserRobotData::setTimestamp(double ts)
{
	_timestamp = ts;
}

void LaserRobotData::setLoggerTimestamp(double ts)
{
	_loggerTimestamp = ts;
}

// void LaserRobotData::setTag(const std::string& tag)
// {
// 	_tag = tag;
// }

void LaserRobotData::setHostname(const std::string& hostname_)
{
	_hostname = hostname_;
}

LaserRobotData::Point2DVector LaserRobotData::cartesian() const
{
	Point2DVector points;
		for (size_t i = 0; i < _laserRobotSensor->ranges().size(); ++i) {
			const double& r = _laserRobotSensor->ranges()[i];
			if (r < _laserRobotSensor->laserParams().maxRange) {
				double alpha = _laserRobotSensor->laserParams().firstBeamAngle + i * _laserRobotSensor->laserParams().angularStep;
				points.push_back(Eigen::Vector2d(cos(alpha) * r, sin(alpha) * r));
			}
		}
		return points;
}




bool LaserRobotDataDrawAction::refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_)
{
	if (!DrawAction::refreshPropertyPtrs(params_))
		return false;
	if (_previousParams)
	{
		_beamsDownsampling = _previousParams->makeProperty<IntProperty>(_typeName + "::BEAMS_DOWNSAMPLING", 10);
		_pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", .05f);
		_maxRange = _previousParams->makeProperty<FloatProperty>(_typeName + "::MAX_RANGE", -1.);
	} 
	else 
	{
		_beamsDownsampling = 0;
		_pointSize = 0;
		_maxRange = 0;
	}
	return true;
}


g2o::HyperGraphElementAction* LaserRobotDataDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element, g2o::HyperGraphElementAction::Parameters* params_)
{
	if (typeid(*element).name()!=_typeName)
			return 0;

		refreshPropertyPtrs(params_);
		if (! _previousParams){
			return this;
		}
		if (_show && !_show->value())
			return this;
		LaserRobotData* that = static_cast<LaserRobotData*>(element);

		LaserRobotData::Point2DVector points = that->cartesian();
		if (_maxRange && _maxRange->value() >= 0 ) {
			// prune the cartesian points;
			LaserRobotData::Point2DVector npoints(points.size());
			int k = 0;
			float r2=_maxRange->value();
			r2 *= r2;
			for (size_t i=0; i<points.size(); i++){
				float x = points[i].x();
				float y = points[i].y();
				if (x*x + y*y < r2) npoints[k++] = points[i];
			}
			points = npoints;
		}
		
		
		glPushMatrix();
		SensorLaserRobot* sensor  = dynamic_cast<SensorLaserRobot*>(that->getSensor());
		const SE2& laserPose = sensor->laserParams().laserPose;
		glTranslatef((float)laserPose.translation().x(), (float)laserPose.translation().y(), 0.f);
		glRotatef((float)RAD2DEG(laserPose.rotation().angle()),0.f,0.f,1.f);
		glBegin(GL_POINTS);
		glColor4f(1.f,0.f,0.f,0.5f);
		int step = 1;
		if (_beamsDownsampling )
			step = _beamsDownsampling->value();
		if (_pointSize) {
			glPointSize(_pointSize->value());
		}
		for (size_t i=0; i<points.size(); i+=step){
			glVertex3f((float)points[i].x(), (float)points[i].y(), 0.f);
		}
		glEnd();
		glPopMatrix();
		return this;
}


// G2O_REGISTER_TYPE(LASER_ROBOT_DATA, LaserRobotData);
// G2O_REGISTER_ACTION(LaserRobotDataDrawAction);