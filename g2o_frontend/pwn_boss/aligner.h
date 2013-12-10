#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "g2o_frontend/pwn_core/aligner.h"

namespace pwn_boss {
  
  class Aligner : public pwn::Aligner, public boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Aligner(int id = -1, boss::IdContext *context = 0);
    virtual ~Aligner() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserializeComplete();
  };

}
