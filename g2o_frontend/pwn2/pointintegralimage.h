#ifndef _PWN_POINTINTEGRALIMAGE_H_
#define _PWN_POINTINTEGRALIMAGE_H_
#include "g2o_frontend/boss_logger/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "pointaccumulator.h"

namespace pwn {

class PointIntegralImage : public Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic> {
 public:
  PointIntegralImage();
  
  void compute(const IntImage &pointIndices, const PointVector &points);
  
  void clear();
  
  PointAccumulator getRegion(int xmin, int xmax, int ymin, int ymax) const;
};

}

#endif
