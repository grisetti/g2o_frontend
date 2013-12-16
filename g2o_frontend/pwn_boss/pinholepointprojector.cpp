#include "pinholepointprojector.h"

namespace pwn_boss {

  PinholePointProjector::PinholePointProjector(int id, boss::IdContext *context) : 
    pwn::PinholePointProjector(),
    PointProjector(id, context) {}

  void PinholePointProjector::serialize(boss::ObjectData &data, boss::IdContext &context) {
    PointProjector::serialize(data, context);
    boss::Identifiable::serialize(data, context);
    _cameraMatrix.toBOSS(data, "cameraMatrix");
    data.setFloat("baseline", baseline());
    data.setFloat("alpha", alpha());
  }

  void PinholePointProjector::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    PointProjector::deserialize(data, context);
    boss::Identifiable::deserialize(data, context);
    _cameraMatrix.fromBOSS(data, "cameraMatrix");
    setBaseline(data.getFloat("baseline"));
    setAlpha(data.getFloat("alpha"));
  }

  BOSS_REGISTER_CLASS(PinholePointProjector);

}
