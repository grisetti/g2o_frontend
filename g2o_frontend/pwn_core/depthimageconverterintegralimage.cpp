#include "depthimageconverterintegralimage.h"
#include "statscalculatorintegralimage.h"
#include <fstream>

namespace pwn {

  DepthImageConverterIntegralImage::DepthImageConverterIntegralImage(PointProjector *projector_,
								     StatsCalculator *statsCalculator_,
								     PointInformationMatrixCalculator *pointInformationMatrixCalculator_,
								     NormalInformationMatrixCalculator *normalInformationMatrixCalculator_) : 
    DepthImageConverter(projector_, 
			statsCalculator_, 
			pointInformationMatrixCalculator_, 
			normalInformationMatrixCalculator_) {}
  
  void DepthImageConverterIntegralImage::compute(Frame &frame,
						 const DepthImage &depthImage, 
						 const Eigen::Isometry3f &sensorOffset) {    
    assert(_projector && "DepthImageConverterIntegralImage: missing _projector");
    assert(_statsCalculator && "DepthImageConverterIntegralImage: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _normalInformationMatrixCalculator");
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "DepthImageConverterIntegralImage: depthImage has zero size");

    StatsCalculatorIntegralImage *statsCalculator = 0;
    statsCalculator = dynamic_cast<StatsCalculatorIntegralImage*>(_statsCalculator);
    assert(statsCalculator && "DepthImageConverterIntegralImage: _statsCalculator of non type StatsCalculatorIntegralImage");

    const float _normalWorldRadius = statsCalculator->worldRadius();
    frame.clear();
    _projector->setImageSize(depthImage.rows, depthImage.cols);
    
    // Resizing the temporaries    
    if (depthImage.rows != _indexImage.rows || depthImage.cols != _indexImage.cols){
      _indexImage.create(depthImage.rows, depthImage.cols);
    }

    // Unprojecting
    _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(frame.points(), frame.gaussians(), _indexImage, depthImage);

    // Computing the intervals
    _projector->projectIntervals(statsCalculator->intervalImage(), depthImage, _normalWorldRadius);

    std::ofstream os("new.txt");
    if(!os) {
      std::cout << "Impossible to open file" << std::endl;
      return;
    }
    std::cout << "Size: " << statsCalculator->intervalImage().rows << " --- " << statsCalculator->intervalImage().cols << std::endl;
    for(int i = 0; i < statsCalculator->intervalImage().rows; i++) {
      for(int j = 0; j < statsCalculator->intervalImage().cols; j++) {
	os << statsCalculator->intervalImage()(i, j) << " "; 
      }
      os << std::endl;
    }

    // Compute stats
    statsCalculator->compute(frame.normals(),
			     frame.stats(),
			     frame.points(),
			     _indexImage);

    // Compute information matrices
    _pointInformationMatrixCalculator->compute(frame.pointInformationMatrix(), frame.stats(), frame.normals());
    _normalInformationMatrixCalculator->compute(frame.normalInformationMatrix(), frame.stats(), frame.normals());

    frame.transformInPlace(sensorOffset);
  }

}
