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

#ifndef IMUDATA_H
#define IMUDATA_H

#include <iosfwd>
#include <string>

#include "g2o/core/hyper_graph.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "sensor_data.h"
#include "sensor_imu.h"

/**
* \brief representation of an imu measurement
*
* An imu measurement obtained by a robot. The measurement is equipped with a pose of the robot at which
* the measurement was taken.
*/
class ImuData : public ParameterizedSensorData
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	typedef Eigen::Matrix<float, 9,1> Vector9f;
	
	ImuData();
	virtual ~ImuData();
	//! read the data from a stream
	virtual bool read(std::istream& is);
	//! write the data to a stream
	virtual bool write(std::ostream& os) const;

	inline Eigen::Quaternionf getOrientation() { return _orientation; };
	inline const Eigen::Quaternionf getOrientation() const { return _orientation; };
	inline void setOrientation(const Eigen::Quaternionf& orientation_)
	{
		_orientation = orientation_;
	}
	
	inline Vector9f getOrientationCovariance() { return _orientationCovariance; };
	inline const Vector9f getOrientationCovariance() const { return _orientationCovariance; };
	inline void setOrientationCovariance(const Vector9f& orientationCovariance_)
	{
		_orientationCovariance = orientationCovariance_;
	}
	
	inline Eigen::Vector3f getAngularVelocity() { return _angularVelocity; };
	inline const Eigen::Vector3f getAngularVelocity() const { return _angularVelocity; };
	inline void setAngularVelocity(const Eigen::Vector3f& angularVelocity_)
	{
		_angularVelocity = angularVelocity_;
	}
	
	inline Vector9f getAngularVelocityCovariance() { return _angularVelocityCovariance; };
	inline const Vector9f getAngularVelocityCovariance() const { return _angularVelocityCovariance; };
	inline void setAngularVelocityCovariance(const Vector9f& angularVelocityCovariance_)
	{
		_angularVelocityCovariance = angularVelocityCovariance_;
	}
	
	inline Eigen::Vector3f getLinearAcceleration() { return _linearAcceleration; };
	inline const Eigen::Vector3f getLinearAcceleration() const { return _linearAcceleration; };
	inline void setLinearAcceleration(const Eigen::Vector3f& linearAcceleration_)
	{ 
		_linearAcceleration = linearAcceleration_;
	}
	
	inline Vector9f getLinearAccelerationCovariance() { return _linearAccelerationCovariance; };
	inline const Vector9f getLinearAccelerationCovariance() const { return _linearAccelerationCovariance; };	
	inline void setLinearAccelerationCovariance(const Vector9f& linearAccelerationCovariance_)
	{
		_linearAccelerationCovariance = linearAccelerationCovariance_;
	}
	
	inline Eigen::Vector3f getMagnetic() { return _magnetic; };
	inline const Eigen::Vector3f getMagnetic() const { return _magnetic; };
	inline void setMagnetic(const Eigen::Vector3f& magnetic_)
	{
		_magnetic = magnetic_;
	}
	
protected:
	Eigen::Quaternionf _orientation;
	Vector9f _orientationCovariance;
	Eigen::Vector3f _angularVelocity;
	Vector9f _angularVelocityCovariance;
	Eigen::Vector3f _linearAcceleration;
	Vector9f _linearAccelerationCovariance;
	Eigen::Vector3f _magnetic;
};


/*inline 	Eigen::Vector3d toVector3D(const Eigen::Isometry3d& iso) {
	
  Eigen::Vector3d rv;
  rv[0] = iso.translation().x();
  rv[1] = iso.translation().y();
  Eigen::AngleAxisd aa(iso.linear());
  rv[2] = aa.angle();	
  return rv;
}


inline 	Eigen::Vector3d toVector3D_fromIso2(const Eigen::Isometry2d& iso) {
	
  Eigen::Vector3d rv;
  rv[0] = iso.translation().x();
  rv[1] = iso.translation().y();
	Eigen::Matrix2d RotMat= iso.rotation();
	double angle = atan2(RotMat(1,0), RotMat(0,0));
//   Eigen::AngleAxisd aa(iso.linear()); //???
  rv[2] = angle;	
  return rv;
  }*/

#endif
