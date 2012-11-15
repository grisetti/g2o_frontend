/*
 * Sensor.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "Sensor.h"

Sensor::Sensor() {
}

Sensor::Sensor(std::string* frame_) {
	_frame = frame_;
}

Sensor::~Sensor() {

}

void Sensor::setFrame( std::string* frame_) {
	_frame = frame_;
}