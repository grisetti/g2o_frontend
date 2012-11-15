/*
 * Sensor.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "Sensor.h"

Sensor::Sensor() {
	_frame = "";
}

Sensor::Sensor(std::string frame_) {
	_frame = frame_;
}

Sensor::~Sensor() {

}
