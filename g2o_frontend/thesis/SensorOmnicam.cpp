/*
 * SensorRGBDCamera.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "SensorOmnicam.h"

SensorOmnicam::SensorOmnicam() {
	_parameter = new g2o::ParameterCamera();
	((g2o::ParameterCamera*)_parameter)->setKcam(5.25, 5.25, 3.195, 2.395);
}

SensorOmnicam::~SensorOmnicam() {

}

bool SensorOmnicam::setParameter(g2o::Parameter* parameter_) {
	g2o::ParameterCamera* camParam = dynamic_cast<g2o::ParameterCamera*>(parameter_);
	if (camParam == 0)
		return false;
	_parameter = parameter_;
	return true;
}

void SensorOmnicam::setTopic(std::string* imageTopic_){
  _topic = imageTopic_;
}

std::string * SensorOmnicam::getTopic(){
  return _topic;
}