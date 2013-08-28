#include "bimusensor.h"

namespace boss {
  IMUSensor::IMUSensor(int id, IdContext* context): BaseSensor(id,context){
  }
  
  IMUData::IMUData(IMUSensor* sensor_, int id, IdContext* context) :SensorData<IMUSensor>(id,context){
    setSensor(sensor_);
  }
 
  IMUData::~IMUData(){
  }

  void IMUData::serialize(ObjectData& data, IdContext& context) {
    SensorData<IMUSensor>::serialize(data,context);
    _orientation.coeffs().toBOSS(data,"orientation");
    _orientationCovariance.toBOSS(data, "orientationCovariance");
    _angularVelocity.toBOSS(data, "angularVelocity");
    _angularVelocityCovariance.toBOSS(data,"angularVelocityCovariance");
    _linearAcceleration.toBOSS(data,"linearAcceleration");
    _linearAccelerationCovariance.toBOSS(data, "linearAccelerationCovariance");
  }

  void IMUData::deserialize(ObjectData& data, IdContext& context) {
    SensorData<IMUSensor>::deserialize(data,context);
    _orientation.coeffs().toBOSS(data,"orientation");
    _orientationCovariance.toBOSS(data, "orientationCovariance");
    _angularVelocity.toBOSS(data, "angularVelocity");
    _angularVelocityCovariance.toBOSS(data,"angularVelocityCovariance");
    _linearAcceleration.toBOSS(data,"linearAcceleration");
    _linearAccelerationCovariance.toBOSS(data, "linearAccelerationCovariance");
  }
  

  BOSS_REGISTER_CLASS(IMUSensor);
  BOSS_REGISTER_CLASS(IMUData);
  
}
