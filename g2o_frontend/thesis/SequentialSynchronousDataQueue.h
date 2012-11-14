/*
 * SequentialSynchronousDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SEQUENTIALSYNCHRONOUSDATAQUEUE_H_
#define SEQUENTIALSYNCHRONOUSDATAQUEUE_H_

#include "SequentialDataQueue.h"

class SequentialSynchronousDataQueue : public SequentialDataQueue {
public:
	SequentialSynchronousDataQueue();
	virtual ~SequentialSynchronousDataQueue();
	virtual void push_back(g2o::HyperGraph::Data* d);
	//fillme
};

#endif /* SEQUENTIALSYNCHRONOUSDATAQUEUE_H_ */
