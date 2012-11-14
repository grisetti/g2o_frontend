/*
 * PrioritySynchronousDataQueue.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "PrioritySynchronousDataQueue.h"

PrioritySynchronousDataQueue::PrioritySynchronousDataQueue() {

}

PrioritySynchronousDataQueue::~PrioritySynchronousDataQueue() {

}

bool PrioritySynchronousDataQueue::empty() {
	_mutex.lock();
	bool isEmpty = _queue.empty();
	_mutex.unlock();
	return(isEmpty);
}

g2o::HyperGraph::Data* PrioritySynchronousDataQueue::front() {
	StampedData* dataPtr = 0;
	{
		_mutex.lock();
		if(!_queue.empty())
			dataPtr = (StampedData*)_queue.top();
		_mutex.unlock();
	}
	return(dataPtr);
}

void PrioritySynchronousDataQueue::pop_front() {
	_mutex.lock();
	_queue.pop();
	_mutex.unlock();
}

void PrioritySynchronousDataQueue::insert(g2o::HyperGraph::Data* d) {
	_mutex.lock();
	_queue.push((StampedData*)d);
	_mutex.unlock();
}
