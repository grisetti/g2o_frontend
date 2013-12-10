#include "statscalculator.h"

namespace pwn_boss {

  StatsCalculator::StatsCalculator(int id, boss::IdContext *context) : 
    pwn::StatsCalculator(), 
    boss::Identifiable(id, context) {}

  void StatsCalculator::serialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::serialize(data, context);
  }
  
  void StatsCalculator::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::deserialize(data, context);
  }

}
