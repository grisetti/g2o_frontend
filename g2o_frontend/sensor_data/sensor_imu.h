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


#ifndef SENSORIMU_H
#define SENSORIMU_H

#include "sensor.h"


class SensorImu: public Sensor
{

public:
	SensorImu();
	virtual ~SensorImu();
		
	virtual g2o::Parameter* getParameter();
	virtual bool setParameter(g2o::Parameter* parameter_);
	
	virtual int getNum();
	virtual void setNum(int num_);
	
	inline std::string getDataTopic() { return _dataTopic; };
//	inline std::string getVelocityTopic() { return _velocityTopic; };
	inline std::string getMagneticTopic() { return _magneticTopic; };
	
	void setDataTopic(const std::string dataTopic_);
//	void setVelocityTopic(const std::string velocityTopic_);
	void setMagneticTopic(const std::string magneticTopic_);
	 
protected:
	std::string _dataTopic;
//	std::string _velocityTopic;
	std::string _magneticTopic;
};

#endif // SENSORIMU_H
