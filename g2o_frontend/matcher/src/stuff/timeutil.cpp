#include "timeutil.h"
#include <iostream>

std::string getTimeAsString(time_t time)
{
  std::string dateStr = ctime(&time);
  if (dateStr.size() == 0)
    return "";
  // remove trailing newline
  dateStr.erase(dateStr.size()-1, 1);
  return dateStr;
}

std::string getCurrentTimeAsString()
{
  return getTimeAsString(time(NULL));
} 


ScopeTime::ScopeTime(const char* title) : _title(title), _startTime(get_monotonic_time()) {}

ScopeTime::~ScopeTime() {
  std::cerr << _title<<" took "<<1000*(get_monotonic_time()-_startTime)<<"ms.\n";
}
