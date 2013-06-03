#ifndef _POINTINTEGRALIMAGE_H_
#define _POINTINTEGRALIMAGE_H_

#include "pointaccumulator.h"

namespace pwn {

class PointIntegralImage : public Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic> {
 public:
  PointIntegralImage();
  
  void compute(const Eigen::MatrixXi &pointIndices, const PointVector &points);
  
  void clear();
  
  PointAccumulator getRegion(int xmin, int xmax, int ymin, int ymax) const;
};

}

#endif
