/*
 * SensorRGBDCamera.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "SensorRGBDCamera.h"

SensorRGBDCamera::SensorRGBDCamera() {

}

SensorRGBDCamera::~SensorRGBDCamera() {

}

bool SensorRGBDCamera::setParameter(g2o::Parameter* p) {
	g2o::ParameterCamera* camParam = dynamic_cast<g2o::ParameterCamera*>(p);
	if (camParam == 0)
		return false;
	_parameter = p;
	return true;
}
