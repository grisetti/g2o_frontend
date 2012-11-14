/*
 * Sensor.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSOR_H_
#define SENSOR_H_

class Sensor {
public:
	Sensor();
	virtual ~Sensor();
	virtual bool setParameter(Parameter* p);
protected:
	Parameter* _parameter;
};

#endif /* SENSOR_H_ */
