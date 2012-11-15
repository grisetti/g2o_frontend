/*
 * Sensor.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "g2o/core/parameter.h"

class Sensor {
public:
	Sensor();
	virtual ~Sensor();
	virtual g2o::Parameter* getParameter();
	virtual bool setParameter(g2o::Parameter* p);
protected:
	g2o::Parameter* _parameter;
};

#endif /* SENSOR_H_ */
