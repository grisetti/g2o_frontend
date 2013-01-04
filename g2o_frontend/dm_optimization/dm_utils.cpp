#include "dm_utils.h"

// Get current time.
clock_t getMilliSecs()
{
  return 1000.0f * ((float)clock() / (float)CLOCKS_PER_SEC)
;
}
