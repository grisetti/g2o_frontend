/*
 * PriorityDataQueue.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "PriorityDataQueue.h"

PriorityDataQueue::PriorityDataQueue() {
  _lastElementTime = -1;
}

bool PriorityDataQueue::empty() const {
  return(_queue.empty());
}

g2o::HyperGraph::Data* PriorityDataQueue::front() {
  return((SensorData*)_queue.top());
}

size_t PriorityDataQueue::size() const {
  return _queue.size();
}

void PriorityDataQueue::pop_front() {
  _queue.pop();
  if (_queue.empty())
    _lastElementTime = -1;
}

double PriorityDataQueue::lastElementTime() const {
  return _lastElementTime;
}

void PriorityDataQueue::insert(g2o::HyperGraph::Data* d_) {
  SensorData* d = (SensorData*) d_;
  _queue.push(d);
  if (_lastElementTime < d->timeStamp())
    _lastElementTime = d->timeStamp();
}
