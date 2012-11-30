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
	std::string getScanTopic() { return _scanTopic; };
	void setScanTopic(std::string scanTopic_);

	//! the range measurements by the laser
	const std::vector<double>& ranges() const { return _ranges; }
	void setRanges(const std::vector<double>& ranges);

	//! the remission measurements by the laser
	const std::vector<double>& intensities() const { return _intensities; }
	void setIntensities(const std::vector<double>& intensities);
	 
protected:
	std::string _scanTopic;
// 	g2o::LaserParameters _laserParams;
	std::vector<double> _ranges;
	std::vector<double> _intensities;

};

#endif // SENSORLASERROBOT_H
