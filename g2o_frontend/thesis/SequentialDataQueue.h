/*
 * SequentialDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SEQUENTIALDATAQUEUE_H_
#define SEQUENTIALDATAQUEUE_H_

class SequentialDataQueue {
public:
	SequentialDataQueue();
	virtual ~SequentialDataQueue();
	virtual void push_back(Data* d);
	//fillme
};

#endif /* SEQUENTIALDATAQUEUE_H_ */
