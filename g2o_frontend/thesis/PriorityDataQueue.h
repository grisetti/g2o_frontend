/*
 * PriorityDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYDATAQUEUE_H_
#define PRIORITYDATAQUEUE_H_

#include "DataQueue.h"
#include "StampedData.h"

class PriorityDataQueue : public DataQueue {
public:
	PriorityDataQueue();
	virtual ~PriorityDataQueue();
	virtual bool empty();
	virtual g2o::HyperGraph::Data* front();
	virtual void pop_front();
	virtual void insert(g2o::HyperGraph::Data* d);
protected:
	std::priority_queue<Stamped*, std::vector<Stamped*>, CompareStamp> _queue;
};

#endif /* PRIORITYDATAQUEUE_H_ */
