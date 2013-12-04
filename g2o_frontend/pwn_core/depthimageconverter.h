#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "frame.h"
#include "pointprojector.h"
#include "statscalculator.h"
#include "informationmatrixcalculator.h"

namespace pwn {

  class DepthImageConverter : public boss::Identifiable {
  public:
    DepthImageConverter(PointProjector *_projector = 0,
			StatsCalculator *_statsCalculator = 0,
			PointInformationMatrixCalculator *_pointInformationMatrixCalculator = 0,
			NormalInformationMatrixCalculator *_normalInformationMatrixCalculator = 0, 
			int id = -1, boss::IdContext *context = 0);

    virtual void compute(Frame &frame,
			 const DepthImage &depthImage, 
			 const Eigen::Isometry3f &sensorOffset = Eigen::Isometry3f::Identity());

    inline PointProjector* projector() { return _projector; }
    inline StatsCalculator* statsCalculator() { return _statsCalculator; }
    inline PointInformationMatrixCalculator* pointInformationMatrixCalculator() { return _pointInformationMatrixCalculator; }
    inline NormalInformationMatrixCalculator* normalInformationMatrixCalculator() { return _normalInformationMatrixCalculator; }
    inline IntImage& indexImage() { return _indexImage; }

    inline void setProjector(PointProjector *projector_) { _projector = projector_; }
    inline void setStatsCalculator(StatsCalculator *statsCalculator_) { _statsCalculator = statsCalculator_; }
    inline void setPointInformationMatrixCalculator(PointInformationMatrixCalculator *pointInformationMatrixCalculator_) { _pointInformationMatrixCalculator = pointInformationMatrixCalculator_; }
    inline void setNormalInformationMatrixCalculator(NormalInformationMatrixCalculator *normalInformationMatrixCalculator_) { _normalInformationMatrixCalculator = normalInformationMatrixCalculator_; }

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserializeComplete();

  protected:
    PointProjector *_projector;
    StatsCalculator *_statsCalculator;
    PointInformationMatrixCalculator *_pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator *_normalInformationMatrixCalculator;  

    IntImage _indexImage;
  };
}
