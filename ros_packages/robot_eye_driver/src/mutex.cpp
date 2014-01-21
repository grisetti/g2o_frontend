#include "mutex.h"

Mutex::Mutex(){
  // std::cout << "creating mutex.";
  pthread_mutex_init(&_mutex, NULL);
}

Mutex::~Mutex(){
  // std::cout << "destroying mutex.";
  pthread_mutex_destroy(&_mutex);
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
