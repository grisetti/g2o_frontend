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
	this->lock();
	bool isEmpty = _queue.empty();
	this->unlock();
	return(isEmpty);
}

g2o::HyperGraph::Data* PrioritySynchronousDataQueue::front() {
	StampedData* dataPtr = 0;
	{
		this->lock();
		if(!_queue.empty())
			dataPtr = (StampedData*)_queue.top();
		this->unlock();
	}
	return(dataPtr);
}

void PrioritySynchronousDataQueue::pop_front() {
	this->lock();
	_queue.pop();
	this->unlock();
}

g2o::HyperGraph::Data* PrioritySynchronousDataQueue::front_and_pop() {
  StampedData* dataPtr = 0;
  this->lock();
  if(!_queue.empty()){
    dataPtr = (StampedData*)_queue.top();
    _queue.pop();
  }
  this->unlock();
  return dataPtr;
}

void PrioritySynchronousDataQueue::insert(g2o::HyperGraph::Data* d) {
	this->lock();
	_queue.push((StampedData*)d);
	this->unlock();
}
