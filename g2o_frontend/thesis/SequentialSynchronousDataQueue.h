/*
 * SequentialSynchronousDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SEQUENTIALSYNCHRONOUSDATAQUEUE_H_
#define SEQUENTIALSYNCHRONOUSDATAQUEUE_H_

#include "SequentialDataQueue.h"
#include "Mutexed.h"

class SequentialSynchronousDataQueue : public SequentialDataQueue, public Mutexed {
public:
	SequentialSynchronousDataQueue();
	~SequentialSynchronousDataQueue();
	bool empty();
	g2o::HyperGraph::Data* front();
	void pop_front();
	void push_back(g2o::HyperGraph::Data* d);
};

#endif /* SEQUENTIALSYNCHRONOUSDATAQUEUE_H_ */
