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
	virtual ~SequentialDataQueue();
	virtual void push_back(g2o::HyperGraph::Data* d);
	//fillme
};

#endif /* SEQUENTIALDATAQUEUE_H_ */
