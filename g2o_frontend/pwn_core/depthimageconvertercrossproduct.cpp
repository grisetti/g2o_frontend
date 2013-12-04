#include "depthimageconvertercrossproduct.h"
#include "statscalculatorcrossproduct.h"

namespace pwn {

  DepthImageConverterCrossProduct::DepthImageConverterCrossProduct(PointProjector *projector_,
								   StatsCalculator *statsCalculator_,
								   PointInformationMatrixCalculator *pointInformationMatrixCalculator_,
								   NormalInformationMatrixCalculator *normalInformationMatrixCalculator_,
								   int id, boss::IdContext *context) : DepthImageConverter(projector_, statsCalculator_, pointInformationMatrixCalculator_, normalInformationMatrixCalculator_, id, context) {}
  
  void DepthImageConverterCrossProduct::compute(Frame &frame,
						const DepthImage &depthImage, 
						const Eigen::Isometry3f &sensorOffset) {
    assert(_projector && "DepthImageConverterCrossProduct: missing _projector");
    assert(_statsCalculator && "DepthImageConverterCrossProduct: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverterCrossProduct: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverterCrossProduct: missing _normalInformationMatrixCalculator");

    StatsCalculatorCrossProduct *statsCalculator = 0;
    statsCalculator = dynamic_cast<StatsCalculatorCrossProduct*>(_statsCalculator);
    assert(statsCalculator && "DepthImageConverterCrossProduct: StatsCalculator of non type StatsCalculatorCrossProduct");

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

  void DepthImageConverterCrossProduct::serialize(boss::ObjectData &data, boss::IdContext &context) {
    DepthImageConverter::serialize(data, context);
  }

  void DepthImageConverterCrossProduct::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    DepthImageConverter::deserialize(data, context);
  }
  
  void DepthImageConverterCrossProduct::deserializeComplete() {
    DepthImageConverter::deserializeComplete();
  }

  BOSS_REGISTER_CLASS(DepthImageConverterCrossProduct);
}
