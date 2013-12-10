#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"

#include "g2o_frontend/pwn_core/multipointprojector.h"

#include "pointprojector.h"

namespace pwn_boss {
  
  class MultiPointProjector : public pwn::MultiPointProjector, public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct ChildProjectorInfo : public boss::Serializable, public pwn::MultiPointProjector::ChildProjectorInfo {
      virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
      virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
      virtual void deserializeComplete();
    };

    MultiPointProjector(int id = -1, boss::IdContext *context = 0);
    virtual ~MultiPointProjector() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
  };

}
