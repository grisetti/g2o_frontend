#include "statscalculatorintegralimage.h"

namespace pwn_boss {

  StatsCalculatorIntegralImage::StatsCalculatorIntegralImage(int id, boss::IdContext *context) : 
    pwn::StatsCalculatorIntegralImage(),
    StatsCalculator(id, context) {}

  void StatsCalculatorIntegralImage::serialize(boss::ObjectData &data, boss::IdContext &context) {
    StatsCalculator::serialize(data, context);
    Identifiable::serialize(data, context);
    data.setFloat("worldRadius", worldRadius());
    data.setInt("imageMaxRadius", maxImageRadius());
    data.setInt("imageMinRadius", minImageRadius());
    data.setInt("minPoints", minPoints());
    data.setFloat("curvatureThreshold", curvatureThreshold());
  }

  void StatsCalculatorIntegralImage::deserialize(boss::ObjectData &data, boss::IdContext &context){
    StatsCalculator::deserialize(data, context);
    Identifiable::deserialize(data, context);
    setWorldRadius(data.getFloat("worldRadius"));
    setMaxImageRadius(data.getInt("imageMaxRadius"));
    setMinImageRadius(data.getInt("imageMinRadius"));
    setMinPoints(data.getInt("minPoints"));
    setCurvatureThreshold(data.getFloat("curvatureThreshold"));
  }

  BOSS_REGISTER_CLASS(StatsCalculatorIntegralImage);

}
