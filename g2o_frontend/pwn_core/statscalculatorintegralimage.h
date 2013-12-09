#pragma once

#include "statscalculator.h"
#include "pointintegralimage.h"

namespace pwn {

  class StatsCalculatorIntegralImage : public StatsCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    StatsCalculatorIntegralImage();
    virtual ~StatsCalculatorIntegralImage() {}

    virtual void compute(NormalVector &normals,
			 StatsVector &statsVector,
			 const PointVector &points,
			 const IntImage &indexImage);
    
    inline void setMaxImageRadius(const int maxImageRadius_) { _maxImageRadius = maxImageRadius_; }
    inline int maxImageRadius() const { return _maxImageRadius; }
    
    inline void setMinImageRadius(const int minImageRadius_) { _minImageRadius = minImageRadius_; }
    inline int minImageRadius() const { return _minImageRadius; }
    
    inline void setMinPoints(const int minPoints_) { _minPoints = minPoints_; }
    inline int minPoints() const { return _minPoints; }
    
    inline void setCurvatureThreshold(float curvatureThreshold_) {_curvatureThreshold = curvatureThreshold_;}
    inline float curvatureThreshold() const { return _curvatureThreshold; }
    
    inline void setWorldRadius(const float worldRadius_) { _worldRadius = worldRadius_; }    
    inline float worldRadius() const { return _worldRadius; }

    inline PointIntegralImage& integralImage() { return _integralImage; }
    inline IntImage& intervalImage() { return _intervalImage; }

  protected:
    int _maxImageRadius;
    int _minImageRadius;
    int _minPoints;
    float _curvatureThreshold;
    float _worldRadius;
    PointIntegralImage _integralImage;
    IntImage _intervalImage;
  };

}
