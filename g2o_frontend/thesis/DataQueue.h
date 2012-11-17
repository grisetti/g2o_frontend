/*
 * DataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef DATAQUEUE_H_
#define DATAQUEUE_H_

#include "g2o/core/hyper_graph.h"
#include <queue>

class DataQueue {
public:
	DataQueue();
	virtual bool empty() = 0;
	virtual g2o::HyperGraph::Data* front() = 0;
	virtual void pop_front() = 0;
};

#endif /* DATAQUEUE_H_ */
