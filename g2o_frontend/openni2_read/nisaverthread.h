#ifndef _NI_SAVER_THREAD_H_
#define _NI_SAVER_THREAD_H_
#include <queue>
#include <pthread.h>

struct Frame;

struct SaverThread {
  std::queue<Frame*> frameList;
  volatile int stopSaving;
  pthread_mutex_t queueMutex;
  pthread_t thread;
  std::string prefix;


  SaverThread(const std::string& prefix);
  void start();
  void stop();
  void addInQueue(Frame* f) ;
  static void* saver(SaverThread* t);

};

#endif
