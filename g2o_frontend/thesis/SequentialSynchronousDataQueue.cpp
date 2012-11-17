/*
 * SequentialSynchronousDataQueue.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "SequentialSynchronousDataQueue.h"

SequentialSynchronousDataQueue::SequentialSynchronousDataQueue() {

}

bool SequentialSynchronousDataQueue::empty() {
	this->lock();
	bool isEmpty = _queue.empty();
	this->unlock();
	return(isEmpty);
}

g2o::HyperGraph::Data* SequentialSynchronousDataQueue::front() {
	g2o::HyperGraph::Data* dataPtr = 0;
	{
		this->lock();
		if(!_queue.empty())
			dataPtr = _queue.front();
		this->unlock();
	}
	return(dataPtr);
}

void SequentialSynchronousDataQueue::pop_front() {
	this->lock();
	_queue.pop();
	this->unlock();
}

void SequentialSynchronousDataQueue::push_back(g2o::HyperGraph::Data* d) {
	this->lock();
	_queue.push(d);
	this->unlock();
}
