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
	Sensor(std::string frame_);
	virtual ~Sensor();
	virtual g2o::Parameter* getParameter();
	virtual bool setParameter(g2o::Parameter* p);
	std::string getFrame() { return _frame; };
protected:
	g2o::Parameter* _parameter;
	std::string _frame;
};

#endif /* SENSOR_H_ */
