/*
 * SensorRGBDCamera.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "SensorRGBDCamera.h"

SensorRGBDCamera::SensorRGBDCamera() {
	_parameter = new g2o::ParameterCamera();
	((g2o::ParameterCamera*)_parameter)->setKcam(5.25, 5.25, 3.195, 2.395);
}

SensorRGBDCamera::~SensorRGBDCamera() {

}

bool SensorRGBDCamera::setParameter(g2o::Parameter* parameter_) {
	g2o::ParameterCamera* camParam = dynamic_cast<g2o::ParameterCamera*>(parameter_);
	if (camParam == 0)
		return false;
	_parameter = parameter_;
	return true;
}

void SensorRGBDCamera::setTopics(string* intensityTopic_, string* depthTopic_) {
	_intensityTopic = intensityTopic_;
	_depthTopic = depthTopic_;
}
