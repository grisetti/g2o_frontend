/*
 * PriorityDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYDATAQUEUE_H_
#define PRIORITYDATAQUEUE_H_

#include "DataQueue.h"
#include "SensorData.h"

class PriorityDataQueue : public DataQueue {
public:
	PriorityDataQueue();
	virtual bool empty() const;
	virtual g2o::HyperGraph::Data* front();
	virtual size_t size() const;
	virtual void pop_front();
	virtual void insert(g2o::HyperGraph::Data* d);
	virtual double lastElementTime() const;

protected:
	std::priority_queue<Stamped*, std::vector<Stamped*>, CompareStamp> _queue;
	double _lastElementTime;
};

#endif /* PRIORITYDATAQUEUE_H_ */
