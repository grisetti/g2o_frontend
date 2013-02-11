/*
 * PrioritySynchronousdata_queue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYSYNCHRONOUSDATAQUEUE_H_
#define PRIORITYSYNCHRONOUSDATAQUEUE_H_

#include "priority_data_queue.h"
#include "mutexed.h"

class PrioritySynchronousDataQueue : public PriorityDataQueue, public Mutexed {
public:
	PrioritySynchronousDataQueue();
	bool empty() const;
	g2o::HyperGraph::Data* front();		// front() and front_and_pop() return 0 if there are no elements in the queue
	g2o::HyperGraph::Data* front_and_pop();	// front_and_pop() pops the first element in the queue, and returns it
	void pop_front();
	void insert(g2o::HyperGraph::Data* d);
	
};

#endif /* PRIORITYSYNCHRONOUSDATAQUEUE_H_ */
