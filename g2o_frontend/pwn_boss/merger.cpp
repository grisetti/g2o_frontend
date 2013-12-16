#include "merger.h"

namespace pwn_boss {

  Merger::Merger(int id, boss::IdContext *context) : 
    pwn::Merger(), 
    boss::Identifiable(id, context) {}

  void Merger::serialize(boss::ObjectData &data, boss::IdContext &context){
    Identifiable::serialize(data, context);
    data.setFloat("distanceThreshold", distanceThreshold());
    data.setFloat("normalThreshold", normalThreshold());
    data.setFloat("maxPointDepth", maxPointDepth());
  }
  
  void Merger::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::deserialize(data, context);
    setDistanceThreshold(data.getFloat("distanceThreshold"));
    setNormalThreshold(data.getFloat("normalThreshold"));
    setMaxPointDepth(data.getFloat("maxPointDepth"));
  }

  BOSS_REGISTER_CLASS(Merger);

}
