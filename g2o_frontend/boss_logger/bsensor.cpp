#include "bsensor.h"
#include "g2o_frontend/boss/object_data.h"

namespace boss {
  BaseSensorData::BaseSensorData(int id, IdContext* context): Identifiable(id,context){}

  BaseSensor::BaseSensor(int id, IdContext* context): Identifiable(id,context) {
    _topic = "unknown";
  }
  
  void BaseSensor::serialize(ObjectData& data, IdContext& /*context*/){
    data.setString("topic", _topic);
  }

  void BaseSensor::deserialize(ObjectData& data, IdContext& /*context*/){
    _topic = data.getString("topic");
  }
  
}
