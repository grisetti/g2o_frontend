#include "informationmatrixcalculator.h"

namespace pwn_boss {

  InformationMatrixCalculator::InformationMatrixCalculator(int id, boss::IdContext *context) : 
    pwn::InformationMatrixCalculator(), 
    boss::Identifiable(id, context) {}

  void InformationMatrixCalculator::serialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::serialize(data, context);
    _flatInformationMatrix.toBOSS(data, "flatInformationMatrix");
    _nonFlatInformationMatrix.toBOSS(data, "nonflatInformationMatrix");
  }
  
  void InformationMatrixCalculator::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::deserialize(data, context);
    _flatInformationMatrix.fromBOSS(data, "flatInformationMatrix");
    _nonFlatInformationMatrix.fromBOSS(data, "nonflatInformationMatrix");
  }

  PointInformationMatrixCalculator::PointInformationMatrixCalculator(int id, boss::IdContext *context) : 
    pwn::PointInformationMatrixCalculator(),
    InformationMatrixCalculator(id, context) {}

  void PointInformationMatrixCalculator::serialize(boss::ObjectData &data, boss::IdContext &context) {
    InformationMatrixCalculator::serialize(data, context);
  }
  
  void PointInformationMatrixCalculator::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    InformationMatrixCalculator::deserialize(data, context);
  }

  NormalInformationMatrixCalculator::NormalInformationMatrixCalculator(int id, boss::IdContext *context) : 
    pwn::NormalInformationMatrixCalculator(),
    InformationMatrixCalculator(id, context) {}

  void NormalInformationMatrixCalculator::serialize(boss::ObjectData &data, boss::IdContext &context) {
    InformationMatrixCalculator::serialize(data, context);
  }
  
  void NormalInformationMatrixCalculator::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    InformationMatrixCalculator::deserialize(data, context);
  }

  BOSS_REGISTER_CLASS(PointInformationMatrixCalculator);
  BOSS_REGISTER_CLASS(NormalInformationMatrixCalculator);
}
