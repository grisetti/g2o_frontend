/*
 * PriorityDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYDATAQUEUE_H_
#define PRIORITYDATAQUEUE_H_

#include "DataQueue.h"

class Stamped {
public:
	double _timeStamp;
};

class StampedData : public g2o::HyperGraph::Data, public Stamped {
};

class CompareTime {
public:
	bool operator()(Stamped* sd1, Stamped* sd2) {
		if(sd1->_timeStamp >= sd2->_timeStamp)
			return true;
		return false;
	}
};

class PriorityDataQueue : public DataQueue {
public:
	PriorityDataQueue();
	virtual ~PriorityDataQueue();
	virtual bool empty();
	virtual g2o::HyperGraph::Data* front();
	virtual void pop_front();
	virtual void insert(g2o::HyperGraph::Data* d);
protected:
	std::priority_queue<Stamped*, std::vector<Stamped*>, CompareTime> _queue;
};

#endif /* PRIORITYDATAQUEUE_H_ */
