#include "bsensor.h"
#include "g2o_frontend/boss/object_data.h"

namespace boss {
  BaseSensorData::BaseSensorData(int id, IdContext* context): Identifiable(id,context){
    _topic = "";
    _robotFrame = 0;
  }

  BaseSensor::BaseSensor(int id, IdContext* context): Identifiable(id,context) {
    _topic = "unknown";
    _frame = 0;
  }
  
  void BaseSensor::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    data.setString("topic", _topic);
    data.setPointer("frame", _frame);
  }

  void BaseSensor::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    _topic = data.getString("topic");
    _frame = 0;
    data.getReference("frame").bind(_frame);
  }

  void BaseSensor::deserializeComplete() {
  }


  
}
