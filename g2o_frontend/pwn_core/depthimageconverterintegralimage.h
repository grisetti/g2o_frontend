#pragma once

#include "depthimageconverter.h"

namespace pwn {

  class DepthImageConverterIntegralImage : public DepthImageConverter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    DepthImageConverterIntegralImage(PointProjector *_projector = 0,
				     StatsCalculator *_statsCalculator = 0,
				     PointInformationMatrixCalculator *_pointInformationMatrixCalculator = 0,
				     NormalInformationMatrixCalculator *_normalInformationMatrixCalculator = 0, 
				     int id = -1, boss::IdContext *context = 0);
    
    virtual void compute(Frame &frame,
			 const DepthImage &depthImage, 
			 const Eigen::Isometry3f &sensorOffset = Eigen::Isometry3f::Identity());

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserializeComplete();
  };
}
