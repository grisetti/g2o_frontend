/*
 * Mutexed.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "Mutexed.h"

Mutexed::Mutexed() {

}

Mutexed::~Mutexed() {

}

void Mutexed::lock() const{
  this->_mutex.lock();
}

void Mutexed::unlock() const{
  this->_mutex.unlock();
}

bool Mutexed::isLocked() const{
  bool getLock = this->_mutex.try_lock();
  if(getLock){
    this->_mutex.unlock();
    return false;
  }
  return true;
}
