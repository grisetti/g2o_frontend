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


#include "sensor_imu.h"
#include <g2o/types/slam3d/parameter_se3_offset.h>

SensorImu::SensorImu()
{
	_num = 0;
	_parameter = new g2o::ParameterSE3Offset();
}


SensorImu::~SensorImu()
{
}


g2o::Parameter* SensorImu::parameter()
{
	return _parameter;
}


bool SensorImu::setParameter(g2o::Parameter* parameter_)
{
	g2o::ParameterSE3Offset* imuParam = dynamic_cast<g2o::ParameterSE3Offset*>(parameter_);
	if(imuParam == 0)
	{
		return false;
	}
	_parameter = parameter_;
	return true;
}


int SensorImu::paramIndex()
{
	return _num;
}


void SensorImu::setNum(int num_)
{
	_num = num_;
}


void SensorImu::setDataTopic(const std::string dataTopic_)
{
	_dataTopic = dataTopic_;
}


//void SensorImu::setVelocityTopic(const std::string velocityTopic_)
//{
//	_velocityTopic = velocityTopic_;
//}


void SensorImu::setMagneticTopic(const std::string magneticTopic_)
{
	_magneticTopic = magneticTopic_;
}

