#include "voxelcalculator.h"

namespace pwn_boss {

  VoxelCalculator::VoxelCalculator(int id, boss::IdContext *context) : 
    pwn::VoxelCalculator(), 
    boss::Identifiable(id, context) {}

  void VoxelCalculator::serialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::serialize(data, context);
    data.setFloat("resolution", resolution());
  }
  void VoxelCalculator::deserialize(boss::ObjectData &data, boss::IdContext &context){
    Identifiable::deserialize(data, context);
    setResolution(data.getFloat("resolution"));
  }

  BOSS_REGISTER_CLASS(VoxelCalculator);

}
