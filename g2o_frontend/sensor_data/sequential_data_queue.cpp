/*
 * sequential_data_queue.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "sequential_data_queue.h"

SequentialDataQueue::SequentialDataQueue() {
	
}

bool SequentialDataQueue::empty() {
	return(_queue.empty());
}

g2o::HyperGraph::Data* SequentialDataQueue::front() {
	return(_queue.front());
}

void SequentialDataQueue::pop_front() {
	_queue.pop();
}

void SequentialDataQueue::push_back(g2o::HyperGraph::Data* d)
{
	_queue.push(d);
}
