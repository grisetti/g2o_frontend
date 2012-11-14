/*
 * PriorityDataQueue.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "PriorityDataQueue.h"

PriorityDataQueue::PriorityDataQueue() {

}

PriorityDataQueue::~PriorityDataQueue() {

}

bool PriorityDataQueue::empty() {
	return(_queue.empty());
}

g2o::HyperGraph::Data* PriorityDataQueue::front() {
		return((StampedData*)_queue.top());
}

void PriorityDataQueue::pop_front() {
	_queue.pop();
}

void PriorityDataQueue::insert(g2o::HyperGraph::Data* d) {
	_queue.push((StampedData*)d);
}
