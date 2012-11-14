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

bool SensorRGBDCamera:: setParameter(Parameter* p) {
	CameraParameter camParam = dynamic_cast<CameraParameter*>(p);
	if (!camParam)
		return false;
	_parameter = p;
	// DA CAMBIARE (VEDERE PARAMETRI DEPTH_REGISTERED)
	_parameter->setKcam(5.75, 5.75, 3.14, 2.35);
	return true;
}
