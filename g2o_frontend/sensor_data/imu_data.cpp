/*
    Sensor handler for XSens IMUs
    Copyright (C) 2012  Taigo M. Bonanni bonanni@dis.uniroma1.it

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

#include "imu_data.h"
#include "g2o/stuff/macros.h"
#include "g2o/core/factory.h"
#include <Eigen/Dense>
#include <fstream>
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
using namespace Eigen;


ImuData::ImuData()
{
	_orientation = Quaternionf(1., .0, .0, .0); // Quaternion constructor is Quaternion(w, x, y, z)
	_orientationCovariance.setZero();
	_angularVelocity.setZero();
	_angularVelocityCovariance.setZero();
	_linearAcceleration.setZero();
	_linearAccelerationCovariance.setZero();
	_magnetic.setZero();
	_imuSensor = 0;
	_paramIndex = -1;
}


ImuData::~ImuData()
{
}


bool ImuData::read(istream& is)
{
	is >> _paramIndex;

	int orientationSize;
	
	// For the first time is equal to 4, that is the size of a Quaternion
	is >> orientationSize;

	// Direct copy of the Quaternion
	is >> _orientation.x() >> _orientation.y() >> _orientation.z() >> _orientation.w();

	is >> orientationSize;
	_orientationCovariance.resize(orientationSize);
	for(int i = 0; i < orientationSize; ++i)
	{
		is >> _orientationCovariance[i];
	}
	
	is >> orientationSize;
	_angularVelocity.resize(orientationSize);
	for(int i = 0; i < orientationSize; ++i)
	{
		is >> _angularVelocity[i];
	}
	
	is >> orientationSize;
	_angularVelocityCovariance.resize(orientationSize);
	for(int i = 0; i < orientationSize; ++i)
	{
		is >> _angularVelocityCovariance[i];
	}
	
	is >> orientationSize;
	_linearAcceleration.resize(orientationSize);
	for(int i = 0; i < orientationSize; ++i)
	{
		is >> _linearAcceleration[i];
	}

	is >> orientationSize;
	_linearAccelerationCovariance.resize(orientationSize);
	for(int i = 0; i < orientationSize; ++i)
	{
		is >> _linearAccelerationCovariance[i];
	}

	is >> orientationSize;
	_magnetic.resize(orientationSize);
	for(int i = 0; i < orientationSize; ++i)
	{
		is >> _magnetic[i];
	}
			
	// imu stuff
	double x,y,theta;
	//odom pose of the robot: no need
	is >> x >> y >> theta;
	//imu pose wrt to the world: no need
	is >> x >> y >> theta;

	// timestamp + hostname
	string hostname;
	double ts;
	is >> ts;
	is >> hostname;
	setTimeStamp(ts);
	is >> ts;
	return true;
}


bool ImuData::write(ostream& os) const
{
	const HyperGraph::DataContainer* container = dataContainer();
	const OptimizableGraph::Vertex* v = dynamic_cast<const OptimizableGraph::Vertex*>(container);
	if(!v)
	{
		cerr << "dataContainer = " << container << endl;
		cerr << "dynamic cast failed" << endl;
		return false;
	}
		
	const OptimizableGraph* g = v->graph();
	if(!g)
	{
		cerr << "no graph" << endl;
		return false;
	}
		
//	const Parameter* p = g->parameters().getParameter(_paramIndex);
//	if(!p) 
//	{
//		cerr << "no param, damn" << endl;
//		return false;
//	}

//	const ParameterSE3Offset* oparam = dynamic_cast<const g2o::ParameterSE3Offset*> (p);
//	if(!oparam)
//	{
//		cerr << "no good param" << endl;
//		return false;
//	}
		
//	os << _paramIndex << " " << 4; // Quaternion size is always equal to 4

	os << paramIndex();
	os << " " << _orientation.x() << " " << _orientation.y() << " " << _orientation.z() << " " << _orientation.w();
	
	os << " " << _orientationCovariance.size();
	for(int i = 0; i < _orientationCovariance.size(); ++i)
	{
		os << " " << _orientationCovariance[i];
	}
	
	os << " " << _angularVelocity.size();
	for(int i = 0; i < _angularVelocity.size(); ++i)
	{
		os << " " << _angularVelocity[i];
	}
	
	os << " " << _angularVelocityCovariance.size();
	for(int i = 0; i < _angularVelocityCovariance.size(); ++i)
	{
		os << " " << _angularVelocityCovariance[i];
	}
	
	os << " " << _linearAcceleration.size();
	for(int i = 0; i < _linearAcceleration.size(); ++i)
	{
		os << " " << _linearAcceleration[i];
	}
	
	os << " " << _linearAccelerationCovariance.size();
	for(int i = 0; i < _linearAccelerationCovariance.size(); ++i)
	{
		os << " " << _linearAccelerationCovariance[i];
	}
	
	os << " " << _magnetic.size();
	for(int i = 0; i < _magnetic.size(); ++i)
	{
		os << " " << _magnetic[i];
	}

//	const VertexSE3* v3 = dynamic_cast<const VertexSE3*>(container);
//	if(v3)
//	{
//		Eigen::Isometry3d offset = oparam->offset();
		// imu pose wrt the world
//		Eigen::Vector3d pose = toVector3D(v3->estimate()*offset);
//		os << " " << pose.x() << " " << pose.y() << " " << pose.z();
		// odometry pose
//		pose = toVector3D(v3->estimate());
//		os << " " << pose.x() << " " << pose.y() << " " << pose.z();
//	}
	
//	const VertexSE2* v2 = dynamic_cast<const VertexSE2*>(container);
//	if(v2)
//	{
//		Eigen::Isometry2d offset;
//		Vector3d ov = toVector3D(oparam->offset());
//		offset.linear() = Rotation2Dd(ov.z()).matrix();
//		offset.translation() = ov.head<2>();
		
//		Eigen::Isometry2d vEstimate;
//		Vector3d ev = v2->estimate().toVector();
//		vEstimate.linear() = Rotation2Dd(ev.z()).matrix();
//		vEstimate.translation() = ev.head<2>();
		// imu pose wrt the world
//		Eigen::Vector3d pose;
// 		pose.setZero();
//		pose = toVector3D_fromIso2(vEstimate*offset); 
//		os << " " << pose.x() << " " << pose.y() << " " << pose.z();
		// odometry pose
//		pose = toVector3D_fromIso2(vEstimate); 
//		os << " " << pose.x() << " " << pose.y() << " " << pose.z();
//	}

	//the second timestamp is the logged timestamp
	string hn = "hostname";
	os << FIXED(" " << _timeStamp << " " << hn << " " << _timeStamp);

	return os.good();
}


void ImuData::setSensor(Sensor* imuSensor_)
{
  _imuSensor = dynamic_cast<SensorImu*>(imuSensor_);
}


G2O_REGISTER_TYPE(IMU_DATA, ImuData);
