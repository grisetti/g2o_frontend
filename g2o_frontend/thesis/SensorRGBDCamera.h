/*
 * SensorRGBDCamera.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORRGBDCAMERA_H_
#define SENSORRGBDCAMERA_H_

#include "Sensor.h"
#include "g2o/types/slam3d/parameter_camera.h"

class SensorRGBDCamera : public Sensor {
public:
	SensorRGBDCamera();
	virtual ~SensorRGBDCamera();
	bool setParameter(g2o::Parameter* p);
};

#endif /* SENSORRGBDCAMERA_H_ */
