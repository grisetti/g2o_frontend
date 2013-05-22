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


#ifndef SENSORHANDLERLASERROBOT_H
#define SENSORHANDLERLASERROBOT_H
#include <g2o_frontend/sensor_data/sensor_handler.h>
#include <g2o_frontend/sensor_data/sensor_laser_robot.h>
#include <g2o_frontend/sensor_data/priority_synchronous_data_queue.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
/*#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>*/

class SensorHandlerLaserRobot: public SensorHandler {
public:
	SensorHandlerLaserRobot(tf::TransformListener* tfListener_);
	virtual ~SensorHandlerLaserRobot();
		
  PriorityDataQueue* getQueue() { return _queue; };
  virtual bool setQueue(PriorityDataQueue* queue_);
	virtual Sensor* getSensor();
	virtual bool setSensor(Sensor* sensor_s);
  void setNodeHandler(ros::NodeHandle* nh_);
  ros::NodeHandle* getNodeHandler() { return _nh; };
  virtual void registerCallback();
  void callback(const sensor_msgs::LaserScanConstPtr& laser_scan);
	
protected:
  void _sensorCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
  void _calibrationCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
	
	ros::Subscriber _scan_sub;
	ros::NodeHandle* _nh;
  tf::TransformListener* _tfListener;
  bool _isCalibrated; // ugly
};

#endif // SENSORHANDLERLASERROBOT_H
