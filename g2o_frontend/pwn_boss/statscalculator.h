#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "g2o_frontend/pwn_core/statscalculator.h"

namespace pwn_boss {
  
  class StatsCalculator : virtual public pwn::StatsCalculator, public boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    StatsCalculator(int id = -1, boss::IdContext *context = 0);
    virtual ~StatsCalculator() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
  };

}
