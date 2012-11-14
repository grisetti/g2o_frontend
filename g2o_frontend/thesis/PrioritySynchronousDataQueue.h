/*
 * PrioritySynchronousDataQueue.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef PRIORITYSYNCHRONOUSDATAQUEUE_H_
#define PRIORITYSYNCHRONOUSDATAQUEUE_H_

class PrioritySynchronousDataQueue {
public:
	PrioritySynchronousDataQueue();
	virtual ~PrioritySynchronousDataQueue();
	virtual void insert(Data* d);
	//fillme
};

#endif /* PRIORITYSYNCHRONOUSDATAQUEUE_H_ */
