#include "depthimageconverterintegralimage.h"
#include "statscalculatorintegralimage.h"

namespace pwn {

  DepthImageConverterIntegralImage::DepthImageConverterIntegralImage(PointProjector *projector_,
								     StatsCalculator *statsCalculator_,
								     PointInformationMatrixCalculator *pointInformationMatrixCalculator_,
								     NormalInformationMatrixCalculator *normalInformationMatrixCalculator_,
								     int id, boss::IdContext *context) : DepthImageConverter(projector_, statsCalculator_, pointInformationMatrixCalculator_, normalInformationMatrixCalculator_, id, context) {}
  
  void DepthImageConverterIntegralImage::compute(Frame &frame,
						 const DepthImage &depthImage, 
						 const Eigen::Isometry3f &sensorOffset) {
    assert(_projector && "DepthImageConverterIntegralImage: missing _projector");
    assert(_statsCalculator && "DepthImageConverterIntegralImage: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _normalInformationMatrixCalculator");

    StatsCalculatorIntegralImage *statsCalculator = 0;
    statsCalculator = dynamic_cast<StatsCalculatorIntegralImage*>(_statsCalculator);
    assert(statsCalculator && "DepthImageConverterIntegralImage: StatsCalculator of non type StatsCalculatorIntegralImage");

    const float _normalWorldRadius = statsCalculator->worldRadius();
    frame.clear();
    _projector->setImageSize(depthImage.rows(), depthImage.cols());
    
    // resizing the temporaries    
    if (depthImage.rows() != _indexImage.rows() ||
	depthImage.cols() != _indexImage.cols()){
      _indexImage.resize(depthImage.rows(), depthImage.cols());
    }

    // unprojecting
    _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(frame.points(), frame.gaussians(), _indexImage, depthImage);

    // computing the intervals
    _projector->projectIntervals(statsCalculator->intervalImage(), depthImage, _normalWorldRadius);

    // Compute stats
    statsCalculator->compute(frame.normals(),
			     frame.stats(),
			     frame.points(),
			     _indexImage);

    // Compute information matrices
    _pointInformationMatrixCalculator->compute(frame.pointInformationMatrix(), frame.stats(), frame.normals());
    _normalInformationMatrixCalculator->compute(frame.normalInformationMatrix(), frame.stats(), frame.normals());

    // for(size_t i = 0; i < frame.pointInformationMatrix().size(); i++) {
    //   std::cout << "pInfo: " << std::endl << frame.pointInformationMatrix()[i] << std::endl;
    //   std::cout << "nInfo: " << std::endl << frame.normalInformationMatrix()[i] << std::endl;
    // }

    frame.transformInPlace(sensorOffset);
  }

  void DepthImageConverterIntegralImage::serialize(boss::ObjectData &data, boss::IdContext &context) {
    DepthImageConverter::serialize(data, context);
  }

  void DepthImageConverterIntegralImage::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    DepthImageConverter::deserialize(data, context);
  }
  
  void DepthImageConverterIntegralImage::deserializeComplete() {
    DepthImageConverter::deserializeComplete();
  }

  BOSS_REGISTER_CLASS(DepthImageConverterIntegralImage);
}
