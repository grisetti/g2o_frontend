/*
 * sensor_handler.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORHANDLER_H_
#define SENSORHANDLER_H_

#include "sensor.h"
#include "priority_data_queue.h"

class SensorHandler {
public:
	SensorHandler();
	virtual Sensor* getSensor() = 0;
	virtual bool setQueue(PriorityDataQueue* queue_) = 0;
	virtual bool setSensor(Sensor* sensor_s) = 0;
	virtual void registerCallback() = 0;
	
protected:
	Sensor* _sensor;
	PriorityDataQueue* _queue;
};

#endif /* SENSORHANDLER_H_ */
