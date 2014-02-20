#ifndef _SIMPLE_TIMER_H_
#define _SIMPLE_TIMER_H_

#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>

class SimpleTimer
{
public:
  
  SimpleTimer(){ reset(); };
  
  void reset()
  {
    struct timeval tv;
    gettimeofday( &tv, NULL );
    time = (uint64_t)tv.tv_sec*1000000 + (uint64_t)tv.tv_usec;
  }
  
  uint64_t elapsedTimeMs()
  {
    struct timeval tv;
    gettimeofday( &tv, NULL );
    uint64_t cur_time = (uint64_t)tv.tv_sec*1000 + (uint64_t)tv.tv_usec/1000;
    
    return cur_time - time/1000;
  }
  
  uint64_t elapsedTimeUs()
  {
    struct timeval tv;
    gettimeofday( &tv, NULL );
    uint64_t cur_time = (uint64_t)tv.tv_sec*1000000 + (uint64_t)tv.tv_usec;
    
    return cur_time - time;
  }

private:
  uint64_t time;
};

#endif
