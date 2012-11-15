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

class Sensor {
public:
	Sensor();
	Sensor(std::string* frame_);
	virtual ~Sensor();
	virtual g2o::Parameter* getParameter();
	std::string* getFrame() { return _frame; };
	virtual bool setParameter(g2o::Parameter* parameter_);
	void setFrame( std::string* frame_);
	
	
	
protected:
	g2o::Parameter* _parameter;
	std::string* _frame;
};

#endif /* SENSOR_H_ */
