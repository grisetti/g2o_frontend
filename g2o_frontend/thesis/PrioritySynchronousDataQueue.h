/*
 * PrioritySynchronousDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYSYNCHRONOUSDATAQUEUE_H_
#define PRIORITYSYNCHRONOUSDATAQUEUE_H_

#include "PriorityDataQueue.h"

class PrioritySynchronousDataQueue : public PriorityDataQueue {
public:
	PrioritySynchronousDataQueue();
	virtual ~PrioritySynchronousDataQueue();
	virtual void insert(g2o::HyperGraph::Data* d);
	//fillme
};

#endif /* PRIORITYSYNCHRONOUSDATAQUEUE_H_ */
