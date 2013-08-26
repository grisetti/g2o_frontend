#include "blasersensor.h"

namespace boss {
  LaserSensor::LaserSensor(int id, IdContext* context): BaseSensor(id,context){
  }
  
  LaserData::LaserData(LaserSensor* sensor_, int id, IdContext* context) :SensorData<LaserSensor>(id,context){
    setSensor(sensor_);
    _minRange = 0;
    _maxRange = 30;
    _fov = M_PI;
  }
 
  LaserData::~LaserData(){
  }

  void LaserData::serialize(ObjectData& data, IdContext& context) {
    SensorData<LaserSensor>::serialize(data,context);
    data.setFloat("minRange", _minRange);
    data.setFloat("maxRange", _maxRange);
    data.setFloat("fov", _fov);
    ArrayData* arr = new ArrayData;
    for (size_t i =0; i<_ranges.size(); i++)
      arr->add(_ranges[i]);
    data.setField("ranges", arr);
    arr = new ArrayData;
    for (size_t i =0; i<_remissions.size(); i++)
      arr->add(_remissions[i]);
    data.setField("remissions", arr);
  }

  void LaserData::deserialize(ObjectData& data, IdContext& context) {
    SensorData<LaserSensor>::deserialize(data,context);
    _minRange = data.getFloat("minRange");
    _maxRange = data.getFloat("maxRange");
    _fov = data.getFloat("fov");
    ArrayData* arr = static_cast<ArrayData*>(data.getField("ranges"));
    _ranges.resize(arr->size());
    for (size_t i =0; i<_ranges.size(); i++)
      _ranges[i]=(*arr)[i].getFloat();
    delete arr;
    arr = static_cast<ArrayData*>(data.getField("remissions"));
    _remissions.resize(arr->size());
    for (size_t i =0; i<_remissions.size(); i++)
      _remissions[i]=(*arr)[i].getFloat();
    delete arr;
  }
  
  
}
