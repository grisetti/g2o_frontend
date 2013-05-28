#include <iostream>
#include "depthimageconverter.h"
#include "g2o/stuff/timeutil.h"

using namespace std;

namespace pwn {

DepthImageConverter::DepthImageConverter(  PointProjector* projector_,
                       StatsFinder* statsFinder_,
                       PointInformationMatrixFinder* pointInformationMatrixFinder_,
                       NormalInformationMatrixFinder* normalInformationMatrixFinder_
                       /*TraversabilityAnalyzer* traversabilityAnalyzer_*/){
  _projector = projector_;
  _statsFinder=statsFinder_;
  _pointInformationMatrixFinder=pointInformationMatrixFinder_;
  _normalInformationMatrixFinder = normalInformationMatrixFinder_;
  //_traversabilityAnalyzer = traversabilityAnalyzer_;
  // wannabe  class parameter
  _normalWorldRadius = 0.1;
  _curvatureThreshold = 0.2;
  
}

void DepthImageConverter::compute(Frame& frame,
				  const Eigen::MatrixXf& depthImage, 
				  const Eigen::Isometry3f& sensorOffset){
  frame.clear();
  // resizing the temporaries
  if (depthImage.rows()!=_indexImage.rows() ||
      depthImage.cols()!=_indexImage.cols()){
    _indexImage.resize(depthImage.rows(), depthImage.cols());
    _integralImage.resize(depthImage.rows(), depthImage.cols());
    _intervalImage.resize(depthImage.rows(), depthImage.cols());
  }
  // unprojecting
  _projector->setTransform(Eigen::Isometry3f::Identity());
  _projector->unProject(frame.points(), _indexImage, depthImage);

  frame.normals().resize(frame.points().size());
  frame.pointInformationMatrix().resize(frame.points().size());
  frame.normalInformationMatrix().resize(frame.points().size());
  frame.stats().resize(frame.points().size());
  std::fill(frame.stats().begin(), frame.stats().end(), PointStats());

  // computing the integral image and the intervals
  _integralImage.compute(_indexImage,frame.points());
  _projector->projectIntervals(_intervalImage,depthImage, _normalWorldRadius);
    
  _statsFinder->compute(frame.normals(),
			frame.stats(),
			frame.points(),
			_integralImage,
			_intervalImage,
			_indexImage,
			_curvatureThreshold);
  _pointInformationMatrixFinder->compute(frame.pointInformationMatrix(), frame.stats(), frame.normals());
  _normalInformationMatrixFinder->compute(frame.normalInformationMatrix(), frame.stats(), frame.normals());
  // frame is labeled, now we need to transform all the elements by considering the position
  // of the sensor
  frame.transformInPlace(sensorOffset);
}

}
