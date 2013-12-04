#include "blasersensor.h"
#include <stdexcept>

namespace boss_map {
  using namespace boss;

  LaserSensor::LaserSensor(int id, IdContext* context): BaseSensor(id,context){
    _minRange = 0;
    _maxRange = 30;
    _fov = M_PI;
    _numBeams = 0;
  }
 
  LaserSensor::~LaserSensor(){}

  void LaserSensor::serialize(ObjectData& data, IdContext& context){
    BaseSensor::serialize(data,context);
    data.setFloat("minRange", _minRange);
    data.setFloat("maxRange", _maxRange);
    data.setFloat("fov", _fov);
    data.setInt("numBeams", _numBeams);
  }
  
  void LaserSensor::deserialize(ObjectData& data, IdContext& context){
    BaseSensor::deserialize(data,context);
    _minRange = data.getFloat("minRange");
    _maxRange = data.getFloat("maxRange");
    _fov = data.getFloat("fov");
    _numBeams = data.getInt("numBeams");
  }
 
  LaserData::LaserData(LaserSensor* sensor_, int id, IdContext* context) :SensorData<LaserSensor>(id,context){
    setSensor(sensor_);
  }
 
  LaserData::~LaserData(){
  }

  void LaserData::serialize(ObjectData& data, IdContext& context) {
    SensorData<LaserSensor>::serialize(data,context);
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
    ArrayData* arr = static_cast<ArrayData*>(data.getField("ranges"));
    _ranges.resize(arr->size());
    for (size_t i =0; i<_ranges.size(); i++)
      _ranges[i]=(*arr)[i].getFloat();
    //delete arr;
    arr = static_cast<ArrayData*>(data.getField("remissions"));
    _remissions.resize(arr->size());
    for (size_t i =0; i<_remissions.size(); i++)
      _remissions[i]=(*arr)[i].getFloat();
    //delete arr;

    int nbeams = _remissions.size()>_ranges.size() ? _remissions.size():_ranges.size();
    if (_sensor->numBeams()==0){
      _sensor->setNumBeams(nbeams);
    }
    if (_sensor->numBeams()!=nbeams)
      throw std::runtime_error("configuration mismatch, the number of beams in a sensor does not match the range reading");
  }
  
  float LaserData::minRange() const {return _sensor->minRange();}
  float LaserData::maxRange() const {return _sensor->maxRange();}
  float LaserData::fov() const {return _sensor->fov();}
  void LaserData::setMinRange(float minRange_) { _sensor->setMinRange(minRange_);}
  void LaserData::setMaxRange(float maxRange_) { _sensor->setMaxRange(maxRange_);}
  void LaserData::setFov(float fov_) { _sensor->setFov(fov_);}
  float LaserData::minAngle() const {return _sensor->minAngle();}
  int LaserData::numBeams() const {return _sensor->numBeams();}
  float LaserData::angularResolution() const { return _sensor->angularResolution();}

  BOSS_REGISTER_CLASS(LaserSensor);
  BOSS_REGISTER_CLASS(LaserData);
  
}
