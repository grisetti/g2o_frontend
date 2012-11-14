/*
 * PriorityDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYDATAQUEUE_H_
#define PRIORITYDATAQUEUE_H_

#include "DataQueue.h"

class PriorityDataQueue : public DataQueue {
public:
	PriorityDataQueue();
	virtual ~PriorityDataQueue();
	virtual void insert(g2o::HyperGraph::Data* d);
	//fillme
};

#endif /* PRIORITYDATAQUEUE_H_ */
