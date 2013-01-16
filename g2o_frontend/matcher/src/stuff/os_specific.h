#ifndef _OS_SPECIFIC_HH_
#define _OS_SPECIFIC_HH_

//defines drand48 and getCurrentTime

#ifdef WINDOWS
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/time.h>
typedef unsigned int uint;
#define drand48() ((double) rand()/(double)RAND_MAX)

#ifdef __cplusplus
extern "C" {
#endif

int vasprintf(char** strp, const char* fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif

#ifdef LINUX
#include <sys/time.h>
// nothing to do on real operating systems
#endif

#endif
