/*
 * SequentialDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SEQUENTIALDATAQUEUE_H_
#define SEQUENTIALDATAQUEUE_H_

#include "DataQueue.h"

class SequentialDataQueue : public DataQueue{
public:
	SequentialDataQueue();
	~SequentialDataQueue();
	virtual bool empty();
	virtual g2o::HyperGraph::Data* front();
	virtual void pop_front();
	virtual void push_back(g2o::HyperGraph::Data* d);
protected:
	std::queue<g2o::HyperGraph::Data*> _queue;
};

#endif /* SEQUENTIALDATAQUEUE_H_ */
