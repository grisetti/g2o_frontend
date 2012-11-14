/*
 * DataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef DATAQUEUE_H_
#define DATAQUEUE_H_

class DataQueue {
public:
	DataQueue();
	virtual ~DataQueue();
	DataQueue();
	virtual ~DataQueue();
	virtual bool empty();
	virtual Data* front();
	virtual void pop_front();
};

#endif /* DATAQUEUE_H_ */
