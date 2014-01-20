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


#endif // MUTEX_H
