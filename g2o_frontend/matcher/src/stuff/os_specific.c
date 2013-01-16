#include "os_specific.h"

#ifdef WINDOWS

int vasprintf(char** strp, const char* fmt, va_list ap){
  int n;
  int size = 100;
  char* p;
  char* np;

  if ((p = (char*)malloc(size * sizeof(char))) == NULL)
    return -1;

  while (1) {
    n = vsnprintf(p, size, fmt, ap);
    if (n > -1 && n < size) {
      *strp = p;
      return n;
    }
    if (n > -1)
      size = n+1;
    else
      size *= 2;
    if ((np = (char*)realloc (p, size * sizeof(char))) == NULL) {
      free(p);
      return -1;
    } else
      p = np;
  }
}


#endif
