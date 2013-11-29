#ifndef _PWN_STATSCALCULATOR_H_
#define _PWN_STATSCALCULATOR_H_

#include "statscalculator.h"
#include "pointintegralimage.h"
#include "informationmatrix.h"

namespace pwn {

  class StatsCalculatorIntegralImage : public StatsCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    StatsCalculatorIntegralImage(int id = -1, boss::IdContext *context = 0);
    virtual ~StatsCalculatorIntegralImage() {}

    virtual void compute(NormalVector &normals,
			 StatsVector &statsVector,
			 const PointVector &points,
			 const IntImage &indexImage);
    
    inline void setMaxImageRadius(const int maxImageRadius_) { _maxImageRadius = maxImageRadius_; }
    inline void setMinImageRadius(const int minImageRadius_) { _minImageRadius = minImageRadius_; }
    inline void setMinPoints(const int minPoints_) { _minPoints = minPoints_; }
    inline void setCurvatureThreshold(float curvatureThreshold_) {_curvatureThreshold = curvatureThreshold_;}
    inline void setWorldRadius(const float worldRadius_) { _worldRadius = worldRadius_; }
    
    inline int maxImageRadius() const { return _maxImageRadius; }
    inline int minImageRadius() const { return _minImageRadius; }
    inline int minPoints() const { return _minPoints; }
    inline float curvatureThreshold() const { return _curvatureThreshold; }
    inline float worldRadius() const { return _worldRadius; }
    inline PointIntegralImage& integralImage() { return _integralImage; }
    inline IntImage& intervalImage() { return _intervalImage; }

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);

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

#endif
