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


#include "SensorHandlerLaserRobot.h"
#include <boost/thread/thread.hpp>
#include <g2o_frontend/sensor_data/laser_robot_data.h>
#include <g2o_frontend/sensor_data/sensor_data.h>
#include <g2o/types/slam3d/se3quat.h>
#include <boost/bind.hpp>

SensorHandlerLaserRobot::SensorHandlerLaserRobot(tf::TransformListener* tfListener_) :  _nh(NULL) 
{
	_tfListener = tfListener_;
	_isCalibrated = false;
}

SensorHandlerLaserRobot::~SensorHandlerLaserRobot() {
}


bool SensorHandlerLaserRobot::setQueue(PriorityDataQueue* queue_) {
	PrioritySynchronousDataQueue* prioritySyncQueue = dynamic_cast<PrioritySynchronousDataQueue*>(queue_);
	if (prioritySyncQueue == 0) return false;
	_queue = queue_;
	return true;
}

Sensor* SensorHandlerLaserRobot::getSensor()
{
	return _sensor;
}


bool SensorHandlerLaserRobot::setSensor(Sensor* sensor_)
{
	SensorLaserRobot* sensorLaser = dynamic_cast<SensorLaserRobot*> (sensor_);
	if (sensorLaser == 0) return false;
	_sensor = sensor_;
	return _sensor;
}

void SensorHandlerLaserRobot::setNodeHandler(ros::NodeHandle* nh_)
{
	_nh = nh_;
}

void SensorHandlerLaserRobot::registerCallback()
{
	SensorLaserRobot* s = static_cast<SensorLaserRobot*> (_sensor);
	_scan_sub = _nh->subscribe<sensor_msgs::LaserScan>(s->getTopic(), 100, boost::bind(&SensorHandlerLaserRobot::callback, this, _1));
	std::cout << "Subscribed to topics: " << std::endl;
	std::cout << "\t" << s->getTopic() << std::endl;
}

void SensorHandlerLaserRobot::callback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
		if(_isCalibrated) {
			_calibrationCallback(laser_scan);
		} else {
			_sensorCallback(laser_scan);
		}
}

g2o::SE3Quat toSE3(tf::StampedTransform& lp)
{
  g2o::SE3Quat pose;
  
  pose.setTranslation(Eigen::Vector3d(lp.getOrigin().x(), lp.getOrigin().y(), lp.getOrigin().z()));
  pose.setRotation(Eigen::Quaterniond(lp.getRotation().w(), lp.getRotation().x(), lp.getRotation().y(), lp.getRotation().z()));
  
  return pose;
}

void SensorHandlerLaserRobot::_calibrationCallback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
	SensorLaserRobot* s = static_cast<SensorLaserRobot*> (getSensor());
	if (!s) {
    ROS_ERROR("No sensor is set");
    return;
  }
  string frame_id = laser_scan->header.frame_id;
	tf::StampedTransform laserPoseTransform;
	try {
		ros::Time timestamp;
		// Get transformation
    _tfListener->lookupTransform("/base_link", frame_id, laser_scan->header.stamp, laserPoseTransform);
		_isCalibrated = true;		
	}
	catch (tf::TransformException & ex) {
		ROS_ERROR("%s", ex.what());
	}
	
	if(_isCalibrated) {
	  g2o::ParameterSE3Offset* laserParam = dynamic_cast<g2o::ParameterSE3Offset*>(s->parameter());
	  assert(laserParam && " parameter  not set ");
	  g2o::SE3Quat pose = toSE3(laserPoseTransform);
	  laserParam->setOffset(pose);
	}	
}

void SensorHandlerLaserRobot::_sensorCallback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
	ROS_WARN_ONCE_NAMED("eval", "First Laser-Data Received");


	std::vector<float> ranges(laser_scan->ranges.size());
	for (size_t i = 0; i < laser_scan->ranges.size(); i++) {
	  double r;
	  if(laser_scan->ranges[i] < laser_scan->range_min)
	    r = laser_scan->range_max;
	  else 
	    r = laser_scan->ranges[i];
	  ranges[i] = r;
	}
	LaserRobotData* data = new LaserRobotData();
	data->setMaxRange(laser_scan->range_max);
	data->setMinRange(laser_scan->range_min);
	data->setFov(laser_scan->angle_max - laser_scan->angle_min);

	data->ranges()=ranges;
	data->setSensor(_sensor);
	data->setParamIndex(_sensor->parameter()->id());
	data->setTimeStamp(laser_scan->header.stamp.toSec());
	_queue->insert(data);
}

