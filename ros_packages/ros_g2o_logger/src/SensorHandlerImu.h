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

#ifndef SENSORHANDLERIMU_H
#define SENSORHANDLERIMU_H

#include <g2o_frontend/sensor_data/sensor_handler.h>
#include <g2o_frontend/sensor_data/sensor_imu.h>
#include <g2o_frontend/sensor_data/priority_synchronous_data_queue.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::Vector3Stamped> ImuSyncPolicy;


class SensorHandlerImu : public SensorHandler
{
public:

	typedef Eigen::Matrix<double, 9,1> Vector9d;
	
	SensorHandlerImu(tf::TransformListener* tfListener_);
	virtual ~SensorHandlerImu();

	inline ros::NodeHandle* getNodeHandler() { return _nh; };
	inline const ros::NodeHandle* getNodeHandler() const { return _nh; };
	inline PriorityDataQueue* getQueue() { return _queue; };
	inline const PriorityDataQueue* getQueue() const { return _queue; };
	
	virtual bool setQueue(PriorityDataQueue* queue_);
	virtual Sensor* getSensor();
	virtual bool setSensor(Sensor* sensor_s);
	void setNodeHandler(ros::NodeHandle* nptr_);
	virtual void registerCallback();
	void callback(const sensor_msgs::ImuConstPtr& imu_data_, const geometry_msgs::Vector3StampedConstPtr& imu_magnetic_);

protected:
	void _sensorCallback(const sensor_msgs::ImuConstPtr& imu_data_, const geometry_msgs::Vector3StampedConstPtr& imu_magnetic_);
	void _calibrationCallback(const sensor_msgs::ImuConstPtr& imu_data_, const geometry_msgs::Vector3StampedConstPtr& imu_magnetic_);

    message_filters::Subscriber<sensor_msgs::Imu>* _data_sub;
	message_filters::Subscriber<geometry_msgs::Vector3Stamped>* _magnetic_sub;

    message_filters::Synchronizer<ImuSyncPolicy>* _sync;

    ros::NodeHandle* _nh;
    tf::TransformListener* _tfListener;
    
    bool _isCalibrated;
};

#endif // SENSORHANDLERIMU_H

