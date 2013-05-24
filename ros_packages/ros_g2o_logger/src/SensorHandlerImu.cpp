/*
    XSens Imu Sensor Handler for ROS
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

#include "SensorHandlerImu.h"
#include <boost/thread/thread.hpp>
#include <g2o_frontend/sensor_data/imu_data.h>
#include <g2o_frontend/sensor_data/sensor_data.h>
#include <vector>

using namespace std;
using namespace Eigen;
typedef Matrix<float, 9,1> Vector9f;
//typedef Matrix<float, 6,1> Vector6f;
//typedef Matrix<float, 3,1> Vector3f;

SensorHandlerImu::SensorHandlerImu(tf::TransformListener* tfListener_) : _data_sub(NULL), _magnetic_sub(NULL),
																			_sync(NULL), _nh(NULL)
{
	_tfListener = tfListener_;
	_isCalibrated = false;
}


SensorHandlerImu::~SensorHandlerImu() {}


Sensor* SensorHandlerImu::getSensor()
{
	return _sensor;
}


bool SensorHandlerImu::setQueue(PriorityDataQueue* queue_)
{
	PrioritySynchronousDataQueue* prioritySyncQueue = dynamic_cast<PrioritySynchronousDataQueue*>(queue_);
	if(prioritySyncQueue == 0)
	{
		return false;
	}
	_queue = queue_;
	return true;
}


bool SensorHandlerImu::setSensor(Sensor* sensor_)
{
	SensorImu* sensorImu = dynamic_cast<SensorImu*> (sensor_);
	if(sensorImu == 0)
	{ 
		return false;
	}
	_sensor = sensor_;
	return _sensor;
}


void SensorHandlerImu::setNodeHandler(ros::NodeHandle* nh_) {
	_nh = nh_;
}


void SensorHandlerImu::registerCallback()
{
	SensorImu* s = dynamic_cast<SensorImu*>(_sensor);

	_data_sub = new message_filters::Subscriber<sensor_msgs::Imu>(*_nh, s->getDataTopic(), 1000);
	_magnetic_sub = new message_filters::Subscriber<geometry_msgs::Vector3Stamped>(*_nh, s->getMagneticTopic(), 1000);

	// ApproximateTime takes a queue size as its constructor argument, hence ImuSyncPolicy(200)
	_sync = new message_filters::Synchronizer<ImuSyncPolicy>(ImuSyncPolicy(2000), *_data_sub, *_magnetic_sub);
	_sync->registerCallback(boost::bind(&SensorHandlerImu::callback, this, _1, _2));
	std::cout << "Subscribed to following topics: " << std::endl;
	std::cout << "\t" << s->getDataTopic() << std::endl;
	std::cout << "\t" << s->getMagneticTopic() << std::endl;
}


void SensorHandlerImu::callback(const sensor_msgs::ImuConstPtr& imu_data_, const geometry_msgs::Vector3StampedConstPtr& imu_magnetic_)
{
	if(!_isCalibrated)
	{
	    _calibrationCallback(imu_data_, imu_magnetic_);
	}
	else
	{
		_sensorCallback(imu_data_, imu_magnetic_);
	}
}


void SensorHandlerImu::_calibrationCallback(const sensor_msgs::ImuConstPtr& imu_data_,
											const geometry_msgs::Vector3StampedConstPtr& imu_magnetic_)
{
	SensorImu* s = static_cast<SensorImu*>(getSensor());
	if(!s)
	{
		ROS_ERROR("No sensor is set");
		return;
	}

	string frame_id = imu_data_->header.frame_id;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped humanReadableAndNiceTransform;
	try
	{
		ros::Time timeStamp;
		// Get transformation
//		_tfListener->lookupTransform("/base_link", frame_id, imu_data_->header.stamp, transform);
		
		//HAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
		//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACK
		_tfListener->lookupTransform("/base_link", "/base_link", imu_data_->header.stamp, transform);
		_isCalibrated = true;
		tf::transformStampedTFToMsg(transform, humanReadableAndNiceTransform);
	}
	catch (tf::TransformException & ex)
	{
    	ROS_ERROR("%s", ex.what());
	} 

	if(_isCalibrated)
	{
		g2o::Vector7d vectorQT;
		vectorQT[0] = humanReadableAndNiceTransform.transform.translation.x;
		vectorQT[1] = humanReadableAndNiceTransform.transform.translation.y;
		vectorQT[2] = humanReadableAndNiceTransform.transform.translation.z;
		vectorQT[3] = humanReadableAndNiceTransform.transform.rotation.x;
		vectorQT[4] = humanReadableAndNiceTransform.transform.rotation.y;
		vectorQT[5] = humanReadableAndNiceTransform.transform.rotation.z;
		vectorQT[6] = humanReadableAndNiceTransform.transform.rotation.w;
		g2o::ParameterSE3Offset* imuParam = dynamic_cast<g2o::ParameterSE3Offset*>(s->parameter());
		assert(imuParam && " parameter  not set" );
		imuParam->setOffset(g2o::internal::fromVectorQT(vectorQT));
	}
}


void SensorHandlerImu::_sensorCallback(const sensor_msgs::ImuConstPtr& imu_data_,
										const geometry_msgs::Vector3StampedConstPtr& imu_magnetic_)
{
	ROS_WARN_ONCE_NAMED("eval", "First IMU-Data Received");

	ImuData* data = new ImuData();

	// Fill orientation data
	Quaterniond q(imu_data_->orientation.w, imu_data_->orientation.x, imu_data_->orientation.y, imu_data_->orientation.z);
	data->setOrientation(q);
	
	// Fill orientation covariance data
	Vector9d qCov;
	for(size_t i = 0; i < imu_data_->orientation_covariance.size(); ++i)
	{
		 qCov[i] = imu_data_->orientation_covariance[i];
	}
	data->setOrientationCovariance(qCov);
	
	// Fill angular velocity data
	Vector3d angVel(imu_data_->angular_velocity.x, imu_data_->angular_velocity.y, imu_data_->angular_velocity.z);
	data->setAngularVelocity(angVel);
	
	// Fill angular velocity covariance data
	Vector9d angVelCov;
	for(size_t i = 0; i < imu_data_->angular_velocity_covariance.size(); ++i)
	{
		 angVelCov[i] = imu_data_->angular_velocity_covariance[i];
	}
	data->setAngularVelocityCovariance(angVelCov);
	
	// Fill linear acceleration data
	Vector3d linAcc(imu_data_->linear_acceleration.x, imu_data_->linear_acceleration.y, imu_data_->linear_acceleration.z);
	data->setLinearAcceleration(linAcc);
	
	// Fill linear acceleration covariance data
	Vector9d linAccCov;
	for(size_t i = 0; i < imu_data_->linear_acceleration_covariance.size(); ++i)
	{
		 linAccCov[i] = imu_data_->linear_acceleration_covariance[i];
	}
	data->setLinearAccelerationCovariance(linAccCov);
	
	// Fill magnetic data
	Vector3d magnetic(imu_magnetic_->vector.x, imu_magnetic_->vector.y, imu_magnetic_->vector.z);
	data->setMagnetic(magnetic);
		
	data->setSensor((SensorImu*) _sensor);
	data->setTimeStamp(imu_data_->header.stamp.toSec());
	_queue->insert(data);
}
