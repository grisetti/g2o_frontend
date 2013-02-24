/*
 * StampedData.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/parameter.h"
#include <iostream>


class Sensor;

class Stamped {
public:
	Stamped() : _timeStamp(-1.) {};
	Stamped(double timeStamp_) { _timeStamp = timeStamp_; };
	inline double timeStamp() const { return _timeStamp; }
	void setTimeStamp(double ts);
protected:	
	double _timeStamp;
};

class SensorData : public g2o::HyperGraph::Data, public Stamped {
public:
  SensorData (double timeStamp=-1) : g2o::HyperGraph::Data(), Stamped(timeStamp) {_sensor=0;}
  //! read the data from a stream
  virtual bool read(std::istream& is) = 0;
	//! write the data to a stream
  virtual bool write(std::ostream& os) const = 0;
 	
  virtual const Sensor* sensor() const {return _sensor;}
  virtual Sensor* sensor() {return _sensor;}
  virtual void    setSensor(Sensor* s) {_sensor=s;}
protected:
  Sensor* _sensor;
};


class ParameterizedSensorData : public SensorData {
public:
  ParameterizedSensorData (double timeStamp=-1) : SensorData(timeStamp), _paramIndex(-1) {}
  virtual ~ParameterizedSensorData();
  //! read the data from a stream
  int paramIndex() const;
  void setParamIndex(int paramIndex_);
  const g2o::Parameter* parameter() const;
protected:
  int _paramIndex;
};

class CompareStamp {
public:
	inline bool operator()(const Stamped* sd1, const Stamped* sd2) {
		if(sd1->timeStamp() >= sd2->timeStamp())
			return true;
		return false;
	}
};

#endif /* SENSORDATA_H_ */
