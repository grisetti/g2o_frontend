/*
 * sensor_rgbd_camera.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORRGBDCAMERA_H_
#define SENSORRGBDCAMERA_H_

#include "sensor.h"
#include "g2o/types/slam3d/parameter_camera.h"

class SensorOmnicam : public Sensor {
public:
	SensorOmnicam();
	virtual ~SensorOmnicam();
	virtual g2o::Parameter* getParameter();
	bool setParameter(g2o::Parameter* parameter_);
	virtual int getNum();
	virtual void setNum(int num_);
	std::string * getTopic();
	void setTopic(std::string* imageTopic_);
protected:
	std::string * _topic;
};

#endif /* SENSORRGBDCAMERA_H_ */
