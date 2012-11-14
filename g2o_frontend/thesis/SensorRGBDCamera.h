/*
 * SensorRGBDCamera.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORRGBDCAMERA_H_
#define SENSORRGBDCAMERA_H_

class SensorRGBDCamera : public Sensor {
public:
	SensorRGBDCamera();
	virtual ~SensorRGBDCamera();
	bool setParameter(Parameter* p);
};

#endif /* SENSORRGBDCAMERA_H_ */
