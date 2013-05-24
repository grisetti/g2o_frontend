#ifndef _DEPTHIMAGECONVERTER_H_
#define _DEPTHIMAGECONVERTER_H_

#include "frame.h"
#include "pointprojector.h"
#include "statsfinder.h"
#include "informationmatrixfinder.h"
//#include "g2o_frontend/traversability/traversability_analyzer.h"

namespace pwn {

class DepthImageConverter {
public:
  DepthImageConverter(  PointProjector* _projector,
            StatsFinder* _statsFinder,
            PointInformationMatrixFinder* _pointInformationMatrixFinder,
            NormalInformationMatrixFinder* _normalInformationMatrixFinder
            /*TraversabilityAnalyzer* _traversabilityAnalyzer = 0*/);

  void compute(Frame& frame,
	       const Eigen::MatrixXf& depthImage, 
	       const Eigen::Isometry3f& sensorOffset=Eigen::Isometry3f::Identity());

  // todo: add accessor methods, separate public and private
  //protected:

  PointProjector* _projector;
  StatsFinder* _statsFinder;
  PointInformationMatrixFinder* _pointInformationMatrixFinder;
  NormalInformationMatrixFinder* _normalInformationMatrixFinder;
  //TraversabilityAnalyzer* _traversabilityAnalyzer;
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
