#include "runtime_error.h"
#include "os_specific.h"
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
using namespace std;

RuntimeError::RuntimeError(const char* fmt, ...) :
  std::exception()
{
  char* auxPtr = NULL;
  va_list arg_list;
  va_start(arg_list, fmt);
  int b = vasprintf(&auxPtr, fmt, arg_list);
  va_end(arg_list);
  if (b > 0)
    _errorMsg = auxPtr;
  else
    _errorMsg = "";
  free(auxPtr);
}

RuntimeError::~RuntimeError() throw()
{
}
