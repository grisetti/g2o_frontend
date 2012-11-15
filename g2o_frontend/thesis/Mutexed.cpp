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

void Mutexed::lock(){
  this->_mutex.lock();
}

void Mutexed::unlock(){
  this->_mutex.unlock();
}

void Mutexed::isLocked(){
  bool getLock = this->_mutex.try_lock();
  if(getLock){
    this->_mutex.unlock();
    return false;
  }
  return true;
}