/*
 * Sensor.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "g2o/core/parameter.h"
#include <list>

using namespace std;

class Sensor {
public:
	Sensor();
	virtual g2o::Parameter* getParameter() = 0;
	virtual bool setParameter(g2o::Parameter* parameter_) = 0;
	virtual int getNum() = 0;
	virtual void setNum(int num_) = 0;
protected:
	g2o::Parameter* _parameter;
	int _num;
};

#endif /* SENSOR_H_ */
