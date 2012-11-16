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
	Sensor(string* frameFrom_, string* frameTo_);
	virtual ~Sensor();
	virtual g2o::Parameter* getParameter();
	string* getFrameFrom() { return _frameFrom; };
	virtual bool setParameter(g2o::Parameter* parameter_);
	void setFrameFrom( string* frameFrom_);
	string* getFrameTo() { return _frameTo; };
	void setFrameTo( string* frameTo_);
	
	
	
protected:
	g2o::Parameter* _parameter;
	string* _frameFrom;
	string* _frameTo;
};

#endif /* SENSOR_H_ */
