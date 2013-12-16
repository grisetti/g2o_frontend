#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "g2o_frontend/pwn_core/voxelcalculator.h"

namespace pwn_boss {
  
  class VoxelCalculator : public pwn::VoxelCalculator, public boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    VoxelCalculator(int id = -1, boss::IdContext *context = 0);
    virtual ~VoxelCalculator() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
  };

}
