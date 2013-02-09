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


#ifndef SENSORLASERROBOT_H
#define SENSORLASERROBOT_H
#include "Sensor.h"

class SensorLaserRobot: public Sensor
{

public:
	SensorLaserRobot();
	virtual ~SensorLaserRobot();
	
	//! the parameters of the laser
	virtual g2o::Parameter* getParameter();
	virtual bool setParameter(g2o::Parameter* parameter_);
// 	const g2o::LaserParameters& laserParams() const { return _laserParams; }
// 	void setLaserParams(const g2o::LaserParameters& laserParams_);
	
	virtual int getNum();
	virtual void setNum(int num_);
	std::string getTopic() { return _scanTopic; };
	void setTopic(std::string scanTopic_);

	 
protected:
	std::string _scanTopic;

};

#endif // SENSORLASERROBOT_H
