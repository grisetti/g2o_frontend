#include "aligner.h"
#include "pointprojector.h"
#include "linearizer.h"
#include "correspondencefinder.h"

namespace pwn_boss {

  Aligner::Aligner(int id, boss::IdContext *context) : 
    pwn::Aligner(), 
    Identifiable(id, context) {}

  void Aligner::serialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::serialize(data, context);
    data.setInt("outerIterations", outerIterations());
    data.setInt("innerIterations", innerIterations());
    pwn::t2v(_referenceSensorOffset).toBOSS(data, "referenceSensorOffset");
    pwn::t2v(_currentSensorOffset).toBOSS(data, "currentSensorOffset");
    PointProjector *projector = dynamic_cast<PointProjector*>(_projector);
    if(!projector) {
      throw std::runtime_error("Impossible to convert pwn::PointProjector to pwn_boss::PointProjector");
    }
    data.setPointer("projector", projector);
    Linearizer *linearizer = dynamic_cast<Linearizer*>(_linearizer);
    if(!linearizer) {
      throw std::runtime_error("Impossible to convert pwn::Linearizer to pwn_boss::Linearizer");
    }
    data.setPointer("linearizer", linearizer);
    CorrespondenceFinder *correspondenceFinder = dynamic_cast<CorrespondenceFinder*>(_correspondenceFinder);
    if(!correspondenceFinder) {
      throw std::runtime_error("Impossible to convert pwn::CorrespondenceFinder to pwn_boss::CorrespondenceFinder");
    }
    data.setPointer("correspondenceFinder", correspondenceFinder);
  }

  void Aligner::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::deserialize(data, context);
    setOuterIterations(data.getInt("outerIterations"));
    setInnerIterations(data.getInt("innerIterations"));
    pwn::Vector6f v;
    v.fromBOSS(data, "referenceSensorOffset");
    _referenceSensorOffset = pwn::v2t(v);
    v.fromBOSS(data, "currentSensorOffset");
    _currentSensorOffset = pwn::v2t(v);
    data.getReference("projector").bind(_projector);
    data.getReference("linearizer").bind(_linearizer);
    data.getReference("correspondenceFinder").bind(_correspondenceFinder);
  }

  void Aligner::deserializeComplete() {}

  BOSS_REGISTER_CLASS(Aligner);

}
