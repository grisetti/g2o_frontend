#ifndef _DEPTHIMAGECONVERTER_H_
#define _DEPTHIMAGECONVERTER_H_

#include "frame.h"
#include "pointprojector.h"
#include "statscalculator.h"
#include "informationmatrixcalculator.h"

namespace pwn {

class DepthImageConverter {
public:
  DepthImageConverter(PointProjector* _projector = 0,
		      StatsCalculator* _statsCalculator = 0,
		      PointInformationMatrixCalculator* _pointInformationMatrixCalculator = 0,
		      NormalInformationMatrixCalculator* _normalInformationMatrixCalculator = 0);

  void compute(Frame& frame,
	       const DepthImage &depthImage, 
	       const Eigen::Isometry3f &sensorOffset=Eigen::Isometry3f::Identity(),
	       const bool blackBorders=false);

  // todo: add accessor methods, separate public and private
  //protected:

  PointProjector* _projector;
  StatsCalculator* _statsCalculator;
  PointInformationMatrixCalculator* _pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator* _normalInformationMatrixCalculator;
  
  // this is the workspace of the object
  PointIntegralImage _integralImage;
  Eigen::MatrixXi _indexImage;
  Eigen::MatrixXi _intervalImage;
  Eigen::MatrixXi _originalIndexImage;
  Eigen::MatrixXf _zBuffer;
  
  // wannabe  class parameter
  float _normalWorldRadius;
  float _curvatureThreshold;

};

}

#endif
