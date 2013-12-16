#include "multipointprojector.h"

#include "g2o_frontend/basemath/bm_se3.h"

namespace pwn_boss {

  MultiPointProjector::MultiPointProjector(int id, boss::IdContext *context) : 
    pwn::MultiPointProjector(),
    PointProjector(id, context) {}

  void MultiPointProjector::ChildProjectorInfo::serialize(boss::ObjectData &data, boss::IdContext &/*context*/) {
    PointProjector *projector = dynamic_cast<PointProjector*>(pointProjector);
    if(!projector) {
      throw std::runtime_error("Impossible to convert pwn::PointProjector to pwn_boss::PointProjector");
    }
    data.setPointer("projector", projector);
    t2v(sensorOffset).toBOSS(data, "sensorOffset");
  }

  void MultiPointProjector::ChildProjectorInfo::deserialize(boss::ObjectData &data, boss::IdContext &/*context*/) {
    data.getReference("projector").bind(pointProjector);
    Vector6f v;
    v.fromBOSS(data,"sensorOffset");
    sensorOffset = v2t(v);
  }

  void MultiPointProjector::ChildProjectorInfo::deserializeComplete() {}

  void MultiPointProjector::serialize(boss::ObjectData &data, boss::IdContext &context){
    PointProjector::serialize(data, context);
    boss::ArrayData *arr = new boss::ArrayData;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      arr->add(&_pointProjectors[i]);
    }
    data.setField("childProjectors",arr);
  }
  
  void MultiPointProjector::deserialize(boss::ObjectData &data, boss::IdContext &context){
    PointProjector::deserialize(data, context);
    boss::ArrayData *arr = dynamic_cast<boss::ArrayData*>(data.getField("childProjectors"));
    clearProjectors();
    for(size_t i = 0; i < arr->size(); i++) {
      ChildProjectorInfo &pp = dynamic_cast<ChildProjectorInfo&>((*arr)[i]);
      _pointProjectors.push_back(pp);
    }
    delete arr;
  }

  BOSS_REGISTER_CLASS(MultiPointProjector);

}
