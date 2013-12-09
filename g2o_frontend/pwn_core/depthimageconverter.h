#pragma once

#include "frame.h"
#include "pointprojector.h"
#include "statscalculator.h"
#include "informationmatrixcalculator.h"

namespace pwn {

  class DepthImageConverter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DepthImageConverter(PointProjector *_projector = 0,
			StatsCalculator *_statsCalculator = 0,
			PointInformationMatrixCalculator *_pointInformationMatrixCalculator = 0,
			NormalInformationMatrixCalculator *_normalInformationMatrixCalculator = 0);
    virtual ~DepthImageConverter() {}

    virtual void compute(Frame &frame,
			 const DepthImage &depthImage, 
			 const Eigen::Isometry3f &sensorOffset = Eigen::Isometry3f::Identity());

    inline PointProjector* projector() { return _projector; }
    inline void setProjector(PointProjector *projector_) { _projector = projector_; }

    inline StatsCalculator* statsCalculator() { return _statsCalculator; }
    inline void setStatsCalculator(StatsCalculator *statsCalculator_) { _statsCalculator = statsCalculator_; }

    inline PointInformationMatrixCalculator* pointInformationMatrixCalculator() { return _pointInformationMatrixCalculator; }
    inline void setPointInformationMatrixCalculator(PointInformationMatrixCalculator *pointInformationMatrixCalculator_) { _pointInformationMatrixCalculator = pointInformationMatrixCalculator_; }

    inline NormalInformationMatrixCalculator* normalInformationMatrixCalculator() { return _normalInformationMatrixCalculator; }
    inline void setNormalInformationMatrixCalculator(NormalInformationMatrixCalculator *normalInformationMatrixCalculator_) { _normalInformationMatrixCalculator = normalInformationMatrixCalculator_; }

    inline IntImage& indexImage() { return _indexImage; }

  protected:
    PointProjector *_projector;
    StatsCalculator *_statsCalculator;
    PointInformationMatrixCalculator *_pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator *_normalInformationMatrixCalculator;  

    IntImage _indexImage;
  };
}
