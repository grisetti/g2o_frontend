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
	SensorHandler(Sensor* sensor_);
	virtual ~SensorHandler();
	virtual Sensor getSensor();
	virtual bool setQueue(PriorityDataQueue* queue_);
	virtual bool setSensor();
	virtual bool calibrateSensor();
	virtual void registerCallback();
	virtual bool writeOut() const = 0;
	
protected:
	Sensor* _sensor;
	PriorityDataQueue* _queue;
};

#endif /* SENSORHANDLER_H_ */
