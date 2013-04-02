#ifndef _HOMOGENEOUSPOINT3INTEGRALIMAGE_H_
#define _HOMOGENEOUSPOINT3INTEGRALIMAGE_H_

#include "homogeneouspoint3faccumulator.h"

struct HomogeneousPoint3fIntegralImage: public Eigen::Matrix<HomogeneousPoint3fAccumulator, Eigen::Dynamic, Eigen::Dynamic>{
  HomogeneousPoint3fIntegralImage();
  void compute(const Eigen::MatrixXi pointIndices, const HomogeneousPoint3fVector& points);
  void clear();
  HomogeneousPoint3fAccumulator getRegion(int xmin, int xmax, int ymin, int ymax) const;
};


#endif
