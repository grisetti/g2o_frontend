/*
 * Sensor.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "Sensor.h"

Sensor::Sensor() {
	_num = 0;
}

g2o::Parameter* Sensor::getParameter() { 
	return _parameter; 
}
	
bool Sensor::setParameter(g2o::Parameter* parameter_) { 
	_parameter = parameter_;
	return true; 
}

void Sensor::setNum(int num_) {
	_num = num_;
}
