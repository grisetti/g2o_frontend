/*
 * SensorHandler.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORHANDLER_H_
#define SENSORHANDLER_H_

#include "Sensor.h"
#include "PriorityDataQueue.h"

class SensorHandler {
public:
	SensorHandler();
	virtual ~SensorHandler();
protected:
	Sensor* _sensor;
	PriorityDataQueue* _queue;
};

#endif /* SENSORHANDLER_H_ */
