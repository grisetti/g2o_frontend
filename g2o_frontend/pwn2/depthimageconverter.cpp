#include "depthimageconverter.h"
#include "pinholepointprojector.h"

#include "g2o/stuff/timeutil.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace boss;

namespace pwn {
  DepthImageConverter::DepthImageConverter(PointProjector *projector_,
					   StatsCalculator *statsCalculator_,
					   PointInformationMatrixCalculator *pointInformationMatrixCalculator_,
					   NormalInformationMatrixCalculator *normalInformationMatrixCalculator_,
					   int id, boss::IdContext *context) : Identifiable(id, context) {
    _projector = projector_;
    _statsCalculator = statsCalculator_;
    _pointInformationMatrixCalculator = pointInformationMatrixCalculator_;
    _normalInformationMatrixCalculator = normalInformationMatrixCalculator_;
  }

  void DepthImageConverter::compute(Frame &frame,
				    const DepthImage &depthImage, 
				    const Eigen::Isometry3f &sensorOffset) {
    frame.clear();
    _projector->setImageSize(depthImage.rows(), depthImage.cols());
    
    assert(_projector && "DepthImageConverterIntegralImage: missing _projector");
    assert(_statsCalculator && "DepthImageConverterIntegralImage: missing _statsCalculator");
    assert(_pointInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _pointInformationMatrixCalculator");
    assert(_normalInformationMatrixCalculator && "DepthImageConverterIntegralImage: missing _normalInformationMatrixCalculator");
    
    // resizing the temporaries
    if (depthImage.rows()!=_indexImage.rows() ||
	depthImage.cols()!=_indexImage.cols()){
      _indexImage.resize(depthImage.rows(), depthImage.cols());
    }

    // unprojecting
    _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(frame.points(), frame.gaussians(), _indexImage, depthImage);
    
    _statsCalculator->compute(frame.normals(),
			      frame.stats(),
			      frame.points(),
			      _indexImage);

    _pointInformationMatrixCalculator->compute(frame.pointInformationMatrix(), frame.stats(), frame.normals());
    _normalInformationMatrixCalculator->compute(frame.normalInformationMatrix(), frame.stats(), frame.normals());

    // for(size_t i = 0; i < frame.pointInformationMatrix().size(); i++) {
    //   std::cout << "pInfo: " << std::endl << frame.pointInformationMatrix()[i] << std::endl;
    //   std::cout << "nInfo: " << std::endl << frame.normalInformationMatrix()[i] << std::endl;
    // }

    frame.transformInPlace(sensorOffset);
  }

  void DepthImageConverter::serialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::serialize(data, context);
    data.setPointer("pointProjector", _projector);
    data.setPointer("statsCalculator", _statsCalculator);
    data.setPointer("pointInfoCalculator", _pointInformationMatrixCalculator);
    data.setPointer("normalInfoCalculator", _normalInformationMatrixCalculator);
  }
  
  void DepthImageConverter::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::deserialize(data, context);
    data.getReference("pointProjector").bind(_projector);
    data.getReference("statsCalculator").bind(_statsCalculator);
    data.getReference("pointInfoCalculator").bind(_pointInformationMatrixCalculator);
    data.getReference("normalInfoCalculator").bind(_normalInformationMatrixCalculator);
  }
  
  void DepthImageConverter::deserializeComplete() {
  }

  BOSS_REGISTER_CLASS(DepthImageConverter);
}
