/*
 * sensor_rgbd_camera.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORRGBDCAMERA_H_
#define SENSORRGBDCAMERA_H_

#include "sensor.h"

class SensorRGBDCamera : public Sensor {
public:
	SensorRGBDCamera();
	virtual g2o::Parameter* getParameter();
	virtual bool setParameter(g2o::Parameter* parameter_);
	virtual int getNum();
	virtual void setNum(int num_);
	std::string getIntensityTopic() { return _intensityTopic; };
	std::string getDepthTopic() { return _depthTopic; };
	void setTopics(std::string intensityTopic_, std::string depthTopic_);
	
protected:
	std::string _intensityTopic;
	std::string _depthTopic;
};

#endif /* SENSORRGBDCAMERA_H_ */
