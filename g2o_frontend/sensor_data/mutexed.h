/*
 * mutexed.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef MUTEXED_H_
#define MUTEXED_H_

#include <boost/thread/mutex.hpp>

class Mutexed {
public:
	Mutexed();
	virtual ~Mutexed();
	void lock() const;
	void unlock() const;
	bool isLocked() const;
protected:
	mutable boost::mutex _mutex;
};

#endif /* MUTEXED_H_ */
