#pragma once

#include "statscalculator.h"

namespace pwn {

  class StatsCalculatorCrossProduct : public StatsCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    StatsCalculatorCrossProduct(int id = -1, boss::IdContext *context = 0);
    virtual ~StatsCalculatorCrossProduct() {}

    virtual void compute(NormalVector &normals,
			 StatsVector &statsVector,
			 const PointVector &points,
			 const IntImage &indexImage);

    inline int imageRadius() const { return _imageRadius; }
    inline int minPoints() const { return _minPoints; }
    inline float normalsTraceThreshold() const { return _normalsTraceThreshold; } 

    inline void setImageRadius(const int imageRadius_) { _imageRadius = imageRadius_; }
    inline void setMinPoints(const int minPoints_) { _minPoints = minPoints_; }
    inline void setNormalTraceThreshold(const int normalsTraceThreshold_) { _normalsTraceThreshold = normalsTraceThreshold_; }

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);

  protected:
    int _imageRadius;
    int _minPoints;
    static const int _annulusSectionDegrees = 90;
    float _normalsTraceThreshold;
  };
}
