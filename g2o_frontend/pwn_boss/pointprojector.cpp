#include "pointprojector.h"

#include "g2o_frontend/basemath/bm_se3.h"

namespace pwn_boss {

  PointProjector::PointProjector(int id, boss::IdContext *context) : 
    pwn::PointProjector(), 
    boss::Identifiable(id, context) {}

  void PointProjector::serialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::serialize(data, context);
    t2v(_transform).toBOSS(data, "transform");
    data.setFloat("minDistance", minDistance());
    data.setFloat("maxDistance", maxDistance());
    data.setInt("imageRows", _imageRows);
    data.setInt("imageCols", _imageCols);
  }

  void PointProjector::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::deserialize(data, context);
    Identifiable::deserialize(data, context);
    Vector6f t;
    t.fromBOSS(data, "transform"); 
    setTransform(v2t(t));
    setMinDistance(data.getFloat("minDistance"));
    setMaxDistance(data.getFloat("maxDistance"));
    int ir = data.getInt("imageRows");
    int ic = data.getInt("imageCols");
    setImageSize(ir, ic);
  }

}
