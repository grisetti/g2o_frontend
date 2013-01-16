#include "stat.h"
#include <cmath>
#include <cstdlib>
#include <assert.h>
#include <stuff/os_specific.h>

namespace AISNavigation{

  double triangularSample(double width, double mean){
    double u=drand48();
    if (u<=0.5)
      u=-0.5+sqrt(u*0.5);
    else
      u=0.5-sqrt((1-u)*0.5);
    return u*2*width+mean;
  };


}; // namespace AISNavigation
