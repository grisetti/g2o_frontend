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


#include "sensor_laser_robot.h"
#include <g2o/types/slam3d/parameter_se3_offset.h>

SensorLaserRobot::SensorLaserRobot() 
// 	: _laserParams(0, 180, -M_PI/2, M_PI/180., 50.,0.1, 0)
{
	_num = 0;	
	_parameter =  new g2o::ParameterSE3Offset();
}

SensorLaserRobot::~SensorLaserRobot()
{
}


g2o::Parameter* SensorLaserRobot::parameter() {
	return _parameter;
}

bool SensorLaserRobot::setParameter(g2o::Parameter* parameter_){
  cerr << __PRETTY_FUNCTION__ << "ERORE" << endl;
  g2o::ParameterSE3Offset* laserParam = dynamic_cast<g2o::ParameterSE3Offset*> (parameter_);
  if (laserParam == 0)
    return false;
  _parameter = parameter_;
  return true;
}

int SensorLaserRobot::paramIndex()
{
	return _num;
}

void SensorLaserRobot::setNum(int num_)
{
	_num = num_;
}

void SensorLaserRobot::setTopic(string scanTopic_)
{
	_scanTopic =  scanTopic_;
}


// void SensorLaserRobot::setLaserParams(const g2o::LaserParameters& laserParams_)
// {
// 	_laserParams = laserParams_;
// }
