/*
 * sensor_rgbd_camera.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "sensor_rgbd_camera.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include <cstdio>

SensorRGBDCamera::SensorRGBDCamera() {
	_parameter = new g2o::ParameterCamera();
	((g2o::ParameterCamera*)_parameter)->setKcam(5.25e02, 5.25e02, 3.195e02, 2.395e02);
	_num = 0;
 _baseFilename = "out";
}

g2o::Parameter* SensorRGBDCamera::parameter() {
	return _parameter;
}

bool SensorRGBDCamera::setParameter(g2o::Parameter* parameter_) {
	g2o::ParameterCamera* camParam = dynamic_cast<g2o::ParameterCamera*>(parameter_);
	if (camParam == 0)
		return false;
	_parameter = parameter_;
	return true;
}

int SensorRGBDCamera::paramIndex()
{ 
	return _num;
}
	
void SensorRGBDCamera::setNum(int num_)
{
	_num = num_;
}

void SensorRGBDCamera::setTopics(string intensityTopic_, string depthTopic_) {
  _intensityTopic = intensityTopic_;
  _depthTopic = depthTopic_;
}

std::string SensorRGBDCamera::getCurrentFilename(){
  char buf[1024];
  sprintf(buf, "%s_%d_%05d", &_baseFilename[0], parameter()->id(), _num);
  return std::string(buf);
}
