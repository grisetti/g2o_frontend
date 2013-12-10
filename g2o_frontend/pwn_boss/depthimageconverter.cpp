#include "depthimageconverter.h"
#include "pointprojector.h"
#include "statscalculator.h"
#include "informationmatrixcalculator.h"

namespace pwn_boss {

  DepthImageConverter::DepthImageConverter(pwn::PointProjector *_projector,
					   pwn::StatsCalculator *_statsCalculator,
					   pwn::PointInformationMatrixCalculator *_pointInformationMatrixCalculator,
					   pwn::NormalInformationMatrixCalculator *_normalInformationMatrixCalculator,
					   int id, boss::IdContext *context) : 
    pwn::DepthImageConverter(_projector,
			     _statsCalculator,
			     _pointInformationMatrixCalculator,
			     _normalInformationMatrixCalculator), 
    boss::Identifiable(id, context) {}

  void DepthImageConverter::serialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::serialize(data, context);
    PointProjector *projector = dynamic_cast<PointProjector*>(_projector);
    if(!projector) {
      throw std::runtime_error("Impossible to convert pwn::PointProjector to pwn_boss::PointProjector");
    }
    data.setPointer("pointProjector", projector);
    StatsCalculator *statsCalculator = dynamic_cast<StatsCalculator*>(_statsCalculator);
    if(!statsCalculator) {
      throw std::runtime_error("Impossible to convert pwn::StatsCalculator to pwn_boss::StatsCalculator");
    }
    data.setPointer("statsCalculator", statsCalculator);
    PointInformationMatrixCalculator *pointInformationMatrixCalculator = dynamic_cast<PointInformationMatrixCalculator*>(_pointInformationMatrixCalculator);
    if(!pointInformationMatrixCalculator) {
      throw std::runtime_error("Impossible to convert pwn::PointInformationMatrixCalculator to pwn_boss::PointInformationMatrixCalculator");
    }
    data.setPointer("pointInfoCalculator", pointInformationMatrixCalculator);
    NormalInformationMatrixCalculator *normalInformationMatrixCalculator = dynamic_cast<NormalInformationMatrixCalculator*>(_normalInformationMatrixCalculator);
    if(!normalInformationMatrixCalculator) {
      throw std::runtime_error("Impossible to convert pwn::NormalInformationMatrixCalculator to pwn_boss::NormalInformationMatrixCalculator");
    }
    data.setPointer("normalInfoCalculator", normalInformationMatrixCalculator);
  }

  void DepthImageConverter::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::deserialize(data, context);
    data.getReference("pointProjector").bind(_projector);
    data.getReference("statsCalculator").bind(_statsCalculator);
    data.getReference("pointInfoCalculator").bind(_pointInformationMatrixCalculator);
    data.getReference("normalInfoCalculator").bind(_normalInformationMatrixCalculator);
  }

  void DepthImageConverter::deserializeComplete() {}

}
