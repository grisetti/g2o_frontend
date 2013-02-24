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
  virtual g2o::Parameter* parameter();
  virtual bool setParameter(g2o::Parameter* parameter_);
  virtual int paramIndex();
  virtual void setNum(int num_);
  virtual int getNum() const {return _num;}
  std::string getIntensityTopic() { return _intensityTopic; };
  std::string getDepthTopic() { return _depthTopic; };
  void setTopics(std::string intensityTopic_, std::string depthTopic_);
  void setBaseFilename(const std::string& baseFilename_="") {_baseFilename =baseFilename_;}
  const std::string& baseFilename() const {return _baseFilename;}
  std::string getCurrentFilename();
protected:
  std::string _intensityTopic;
  std::string _depthTopic;
  std::string _baseFilename;
};

#endif /* SENSORRGBDCAMERA_H_ */
