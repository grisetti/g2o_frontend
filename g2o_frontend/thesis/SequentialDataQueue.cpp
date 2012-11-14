/*
 * SequentialDataQueue.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "SequentialDataQueue.h"

SequentialDataQueue::SequentialDataQueue() {
	
}

SequentialDataQueue::~SequentialDataQueue() {

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
