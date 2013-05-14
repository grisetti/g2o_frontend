#ifndef _DEPTHIMAGECONVERTER_H_
#define _DEPTHIMAGECONVERTER_H_

#include "frame.h"
#include "pointprojector.h"
#include "statsfinder.h"
#include "informationmatrixfinder.h"


class DepthImageConverter {
public:
  DepthImageConverter(  PointProjector* _projector,
            StatsFinder* _statsFinder,
            PointInformationMatrixFinder* _pointInformationMatrixFinder,
            NormalInformationMatrixFinder* _normalInformationMatrixFinder );

  void compute(Frame& frame,
	       const Eigen::MatrixXf& depthImage, 
	       const Eigen::Isometry3f& sensorOffset=Eigen::Isometry3f::Identity());

  // todo: add accessor methods, separate public and private
  //protected:

  PointProjector* _projector;
  StatsFinder* _statsFinder;
  PointInformationMatrixFinder* _pointInformationMatrixFinder;
  NormalInformationMatrixFinder* _normalInformationMatrixFinder;
  // this is the workspace of the object
  PointIntegralImage _integralImage;
  Eigen::MatrixXi _indexImage;
  Eigen::MatrixXi _intervalImage;
  Eigen::MatrixXi _originalIndexImage;
  Eigen::MatrixXf _zBuffer;
  
  // wannabe  class parameter
  static const float _normalWorldRadius = 0.1;
  static const float _curvatureThreshold = 0.02;

};

#endif
