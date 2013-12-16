#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "g2o_frontend/pwn_core/informationmatrixcalculator.h"

namespace pwn_boss {

  class InformationMatrixCalculator : virtual pwn::InformationMatrixCalculator, public boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    InformationMatrixCalculator(int id = -1, boss::IdContext *context = 0);
    virtual ~InformationMatrixCalculator() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
  };

  class PointInformationMatrixCalculator : public pwn::PointInformationMatrixCalculator, public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointInformationMatrixCalculator(int id = -1, boss::IdContext *context = 0);
    virtual ~PointInformationMatrixCalculator() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
  };

  class NormalInformationMatrixCalculator : public pwn::NormalInformationMatrixCalculator, public InformationMatrixCalculator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    NormalInformationMatrixCalculator(int id = -1, boss::IdContext *context = 0);
    virtual ~NormalInformationMatrixCalculator() {}

    virtual void serialize(boss::ObjectData &data, boss::IdContext &context);
    virtual void deserialize(boss::ObjectData &data, boss::IdContext &context);
  };

}
