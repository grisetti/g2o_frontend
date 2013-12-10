#pragma once 

#include "pointaccumulator.h"

namespace pwn {

  class PointIntegralImage : public Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointIntegralImage() : Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic>(0, 0) {}
    virtual ~PointIntegralImage() {}

    void compute(const IntImage &pointIndices, const PointVector &points);
  
    void clear();
  
    PointAccumulator getRegion(int xmin, int xmax, int ymin, int ymax);
    
  protected:
    inline int _clamp(int v, int min, int max);
  };

}
