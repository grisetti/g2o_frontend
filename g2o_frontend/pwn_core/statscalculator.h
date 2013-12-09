#pragma once

#include "stats.h"
#include "pwn_typedefs.h"

namespace pwn {

  class StatsCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    StatsCalculator() {}
    virtual ~StatsCalculator() {}

    virtual void compute(NormalVector &normals,
			 StatsVector &statsVector,
			 const PointVector &points,
			 const IntImage &indexImage);
  };

}
