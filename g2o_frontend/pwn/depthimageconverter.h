#ifndef _DEPTHIMAGECONVERTER_H_
#define _DEPTHIMAGECONVERTER_H_

#include "homogeneouspoint3fscene.h"
#include "pointprojector.h"
#include "homogeneouspoint3fstatsgenerator.h"
#include "omegagenerator.h"


class DepthImageConverter{
public:
  DepthImageConverter(  PointProjector* _projector,
			HomogeneousPoint3fStatsGenerator* _statsGenerator,
			PointOmegaGenerator* _pointOmegaGenerator,
			NormalOmegaGenerator*_normalOmegaGenerator );

  void compute(HomogeneousPoint3fScene& scene, 
	       const Eigen::MatrixXf& depthImage, 
	       const Eigen::Isometry3f& sensorOffset=Eigen::Isometry3f::Identity());

  // todo: add accessor methods, separate public and private
  //protected:

  PointProjector* _projector;
  HomogeneousPoint3fStatsGenerator* _statsGenerator;
  PointOmegaGenerator* _pointOmegaGenerator;
  NormalOmegaGenerator*_normalOmegaGenerator;
  // this is the workspace of the object
  HomogeneousPoint3fIntegralImage _integralImage;
  Eigen::MatrixXi _indexImage;
  Eigen::MatrixXi _intervalImage;
  Eigen::MatrixXi _originalIndexImage;
  Eigen::MatrixXf _zBuffer;
  
  // wannabe  class parameter
  static const float _normalWorldRadius = 0.1;
  static const float _curvatureThreshold = 0.02;

};

#endif
