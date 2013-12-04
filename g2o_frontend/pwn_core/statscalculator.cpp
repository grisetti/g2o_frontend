#include "statscalculator.h"

using namespace boss;

namespace pwn {

  StatsCalculator::StatsCalculator(int id, 
				   boss::IdContext *context) : Identifiable(id, context) {}

  void StatsCalculator::compute(NormalVector &normals,
				StatsVector &statsVector,
				const PointVector &points,
				const IntImage &indexImage) {
    assert(indexImage.rows() > 0 && "StatsCalculator: indexImage has zero rows");
    assert(indexImage.cols() > 0 && "StatsCalculator: indexImage has zero columns");
    
    if(statsVector.size() != points.size())
      statsVector.resize(points.size());
    if(normals.size() != points.size())
      normals.resize(points.size());
    Normal dummyNormal = Normal::Zero();
    std::fill(statsVector.begin(), statsVector.end(), Stats());
    std::fill(normals.begin(), normals.end(), dummyNormal);
  }

  void StatsCalculator::serialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::serialize(data, context);
  }

  void StatsCalculator::deserialize(boss::ObjectData &data, boss::IdContext &context){
    Identifiable::deserialize(data, context);
  }
  
  BOSS_REGISTER_CLASS(StatsCalculator);
}
