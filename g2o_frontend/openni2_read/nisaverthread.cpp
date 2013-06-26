#include "niframe.h"
#include "nisaverthread.h"
#include <iostream>
using namespace std;


  SaverThread::SaverThread(const std::string& prefix){
    stopSaving = false;
    pthread_mutex_init(&queueMutex, 0);
    this->prefix = prefix;
  }

  void SaverThread::start(){
    stopSaving = false;
    pthread_mutex_init(&queueMutex, 0);
    pthread_create(&thread, 0, (void *(*) (void *)) saver, (void*) this);
  }

  void SaverThread::stop(){
    stopSaving = true;
    pthread_join(thread, 0);
  }

  void SaverThread::addInQueue(Frame* f) {
    pthread_mutex_lock(&queueMutex);
    frameList.push(f);
    pthread_mutex_unlock(&queueMutex);
  }

  void* SaverThread::saver(SaverThread* t){
    cerr << "saver started" << endl;
    while (! t->stopSaving) {
      while(! t->frameList.empty()){
	pthread_mutex_lock(&t->queueMutex);
	Frame* frame = t->frameList.front();
	t->frameList.pop();
	pthread_mutex_unlock(&t->queueMutex);
	frame->save(t->prefix.c_str());
	cerr << frame->type;
	delete frame;
      }
      usleep(1000);
    }
    cerr << "saver stopped" << endl;
    return 0;
  };
