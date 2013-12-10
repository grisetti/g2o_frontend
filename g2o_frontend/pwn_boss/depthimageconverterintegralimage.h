#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "g2o_frontend/pwn_core/depthimageconverterintegralimage.h"

#include "depthimageconverter.h"

namespace pwn_boss {
  
  class DepthImageConverterIntegralImage : public pwn::DepthImageConverterIntegralImage, public DepthImageConverter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    DepthImageConverterIntegralImage(pwn::PointProjector *_projector = 0,
                                     pwn::StatsCalculator *_statsCalculator = 0,
                                     pwn::PointInformationMatrixCalculator *_pointInformationMatrixCalculator = 0,
                                     pwn::NormalInformationMatrixCalculator *_normalInformationMatrixCalculator = 0, 
				     int id = -1, boss::IdContext *context = 0);
    virtual ~DepthImageConverterIntegralImage() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserializeComplete();
  };

}
