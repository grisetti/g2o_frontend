/*
 * PrioritySynchronousDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYSYNCHRONOUSDATAQUEUE_H_
#define PRIORITYSYNCHRONOUSDATAQUEUE_H_

#include "PriorityDataQueue.h"
#include "Mutexed.h"

class PrioritySynchronousDataQueue : public PriorityDataQueue, public Mutexed {
public:
	PrioritySynchronousDataQueue();
	virtual ~PrioritySynchronousDataQueue();
	bool empty();
	g2o::HyperGraph::Data* front();
	void pop_front();
	void insert(g2o::HyperGraph::Data* d);
};

#endif /* PRIORITYSYNCHRONOUSDATAQUEUE_H_ */
