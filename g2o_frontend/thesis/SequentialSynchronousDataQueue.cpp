/*
 * SequentialSynchronousDataQueue.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "SequentialSynchronousDataQueue.h"

SequentialSynchronousDataQueue::SequentialSynchronousDataQueue() {

}

SequentialSynchronousDataQueue::~SequentialSynchronousDataQueue() {

}

bool SequentialSynchronousDataQueue::empty() {
	_mutex.lock();
	bool isEmpty = _queue.empty();
	_mutex.unlock();
	return(isEmpty);
}

g2o::HyperGraph::Data* SequentialSynchronousDataQueue::front() {
	g2o::HyperGraph::Data* dataPtr = 0;
	{
		_mutex.lock();
		if(!_queue.empty())
			dataPtr = _queue.front();
		_mutex.unlock();
	}
	return(dataPtr);
}

void SequentialSynchronousDataQueue::pop_front() {
	_mutex.lock();
	_queue.pop();
	_mutex.unlock();
}

void SequentialSynchronousDataQueue::push_back(g2o::HyperGraph::Data* d) {
	_mutex.lock();
	_queue.push(d);
	_mutex.unlock();
}
