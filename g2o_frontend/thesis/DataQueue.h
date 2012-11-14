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
	virtual ~DataQueue();
	virtual bool empty();
	virtual g2o::HyperGraph::Data* front();
	virtual void pop_front();
};

#endif /* DATAQUEUE_H_ */
