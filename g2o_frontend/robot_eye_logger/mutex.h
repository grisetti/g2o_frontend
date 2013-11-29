#ifndef MUTEX_H
#define MUTEX_H

#include <pthread.h>
#include <iostream>

class Mutex{
  pthread_mutex_t _mutex;

 public:
  Mutex();
  ~Mutex();
  bool lock();
  bool unlock();
};

Mutex::Mutex(){
  // std::cout << "creating mutex.";
  while(pthread_mutex_init(&_mutex, NULL) == -1){
    std::cout << '.';
    usleep(2e4);
  }
  // std::cout << "\tDONE" << std::endl;
}

Mutex::~Mutex(){
  // std::cout << "destroying mutex.";
  while(pthread_mutex_destroy(&_mutex) == -1){
    // std::cout << '.';
    usleep(2e4);
  }
  // std::cout << "\tDONE" << std::endl;
}

bool Mutex::lock(){
  int mustbezero =  pthread_mutex_lock(&_mutex);
  if(mustbezero == 0){
    return true;
  }
  return false;
}

bool Mutex::unlock(){
  int mustbezero = pthread_mutex_unlock(&_mutex);
  if(mustbezero == 0){
    return true;
  }
  return false;
}


#endif // MUTEX_H
