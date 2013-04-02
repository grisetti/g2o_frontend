#ifndef _HOMOGENEOUSPOINT3FSTATSGENERATOR_H_
#define _HOMOGENEOUSPOINT3FSTATSGENERATOR_H_
#include "homogeneouspoint3fstats.h"
#include "homogeneouspoint3fintegralimage.h"
#include "pointprojector.h"

class HomogeneousPoint3fStatsGenerator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  HomogeneousPoint3fStatsGenerator();
  void compute(HomogeneousPoint3fStatsVector& stats, 
	       const HomogeneousPoint3fIntegralImage& _integralImage,
	       const Eigen::MatrixXi& _intervalImage,
	       /*const Eigen::MatrixXf& depthImage,*/
	       const Eigen::MatrixXi& _indexImage);
  protected:
  float _worldRadius;
  int _maxImageRadius;
  int _minImageRadius;
  int _minPoints;
};

#endif
