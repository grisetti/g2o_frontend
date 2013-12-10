#include "cylindricalpointprojector.h"

namespace pwn_boss {

  CylindricalPointProjector::CylindricalPointProjector(int id, boss::IdContext *context) : 
    pwn::CylindricalPointProjector(),
    PointProjector(id, context) {}

  void CylindricalPointProjector::serialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::serialize(data, context);
    PointProjector::serialize(data,context);
    data.setFloat("angularFov", angularFov());
    data.setFloat("angularResolution", angularResolution());
    data.setFloat("fy", verticalFocalLenght());
    data.setFloat("cy", verticalCenter());
  }

  void CylindricalPointProjector::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::deserialize(data, context);
    setAngularFov(data.getFloat("angularFov"));
    setAngularResolution(data.getFloat("angularResolution"));
    setVerticalFocalLenght(data.getFloat("fy"));
    setVerticalCenter(data.getFloat("cy"));
  }

  BOSS_REGISTER_CLASS(CylindricalPointProjector);

}
