/*
 * SequentialSynchronousDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef SEQUENTIALSYNCHRONOUSDATAQUEUE_H_
#define SEQUENTIALSYNCHRONOUSDATAQUEUE_H_

class SequentialSynchronousDataQueue {
public:
	SequentialSynchronousDataQueue();
	virtual ~SequentialSynchronousDataQueue();
	virtual void push_back(Data* d);
	//fillme
};

#endif /* SEQUENTIALSYNCHRONOUSDATAQUEUE_H_ */
