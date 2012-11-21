/*
 * SensorRGBDCamera.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORRGBDCAMERA_H_
#define SENSORRGBDCAMERA_H_

#include "Sensor.h"
#include "g2o/types/slam3d/parameter_camera.h"

class SensorOmnicam : public Sensor {
public:
	SensorOmnicam();
	virtual ~SensorOmnicam();
	g2o::Parameter* getParameter() const { return _parameter; }
	bool setParameter(g2o::Parameter* parameter_);
	void setTopic(std::string* imageTopic_);
	std::string * getTopic();
	inline void setSensorNumber(int sNum_){_sensorNumber = sNum_;};
	inline int getSensorNumber(){return _sensorNumber;};
protected:
	std::string * _topic;
	int _sensorNumber;
};

#endif /* SENSORRGBDCAMERA_H_ */
