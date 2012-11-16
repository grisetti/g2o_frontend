/*
 * Sensor.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "Sensor.h"

Sensor::Sensor() {
}

Sensor::Sensor(string* frameFrom_, string* framTo_) {
	_frameFrom = frameFrom_;
	_frameTo = framTo_;
}

Sensor::~Sensor() {

}

void Sensor::setFrameFrom( string* frameFrom_) {
	_frameFrom = frameFrom_;
}

void Sensor::setFrameTo(string* frameTo_) {
	_frameTo = frameTo_;
}
