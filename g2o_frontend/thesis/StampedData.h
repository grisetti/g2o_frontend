/*
 * StampedData.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef STAMPEDDATA_H_
#define STAMPEDDATA_H_

#include "g2o/core/hyper_graph.h"

class Stamped {
public:
	Stamped() : _timeStamp(-1.) {};
	Stamped(double timeStamp_) { _timeStamp = timeStamp_; };
	inline double timeStamp() const { return _timeStamp; }
	void setTimestamp(double ts);
protected:	
	double _timeStamp;
};

class StampedData : public g2o::HyperGraph::Data, public Stamped {
public:
	StampedData() : g2o::HyperGraph::Data(), Stamped() {};
	StampedData (double timeStamp) : g2o::HyperGraph::Data(), Stamped(timeStamp) {};
	virtual ~StampedData();
};

class CompareStamp {
public:
	inline bool operator()(const Stamped* sd1, const Stamped* sd2) {
		if(sd1->timeStamp() >= sd2->timeStamp())
			return true;
		return false;
	}
};

#endif /* STAMPEDDATA_H_ */
