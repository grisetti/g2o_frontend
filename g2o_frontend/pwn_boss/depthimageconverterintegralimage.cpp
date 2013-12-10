#include "depthimageconverterintegralimage.h"

namespace pwn_boss {

  DepthImageConverterIntegralImage::DepthImageConverterIntegralImage(pwn::PointProjector *_projector,
								     pwn::StatsCalculator *_statsCalculator,
								     pwn::PointInformationMatrixCalculator *_pointInformationMatrixCalculator,
								     pwn::NormalInformationMatrixCalculator *_normalInformationMatrixCalculator,
								     int id, boss::IdContext *context) : 
    pwn::DepthImageConverterIntegralImage(_projector,
					  _statsCalculator,
					  _pointInformationMatrixCalculator,
					  _normalInformationMatrixCalculator),
    DepthImageConverter(_projector,
			_statsCalculator,
			_pointInformationMatrixCalculator,
			_normalInformationMatrixCalculator,
			id, context) {}

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
