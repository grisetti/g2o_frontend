/*
 * StampedData.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#include "g2o/core/hyper_graph.h"
#include <iostream>


class Sensor;

class Stamped {
public:
	Stamped() : _timeStamp(-1.) {};
	Stamped(double timeStamp_) { _timeStamp = timeStamp_; };
	inline double timeStamp() const { return _timeStamp; }
	void setTimestamp(double ts);
protected:	
	double _timeStamp;
};

class SensorData : public g2o::HyperGraph::Data, public Stamped {
public:
	SensorData() : g2o::HyperGraph::Data(), Stamped() {};
	SensorData (double timeStamp) : g2o::HyperGraph::Data(), Stamped(timeStamp) {};
	//! read the data from a stream
	virtual bool read(std::istream& is) = 0;
	//! write the data to a stream
	virtual bool write(std::ostream& os) const = 0;
	virtual void writeOut(std::string g2oGraphFilename) = 0;
	
	virtual Sensor* getSensor() const = 0;
	virtual void    setSensor(Sensor* s) = 0;

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
