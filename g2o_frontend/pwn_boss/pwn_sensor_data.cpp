#include "pwn_sensor_data.h"
namespace pwn_boss{
  using namespace boss;
  using namespace pwn;
  using namespace boss_logger;


  PWNSensorData::PWNSensorData(int id, IdContext* context): BaseSensorData(id,context){}
    
  void PWNSensorData::serialize(ObjectData& data, IdContext& context) {
    BaseSensorData::serialize(data,context);
    _blob.serialize(data,context);
  }
  void PWNSensorData::deserialize(ObjectData& data, IdContext& context){
    BaseSensorData::deserialize(data,context);
    _blob.deserialize(data,context);
  }

  PWNDepthConverterSensor::PWNDepthConverterSensor(DepthImageConverter* converter_, 
						   int id, 
						   IdContext* context):
    BaseSensor(id,context){
    _converter=converter_;
  }

  void PWNDepthConverterSensor::serialize(ObjectData& data, IdContext& context){
    BaseSensor::serialize(data,context);
    data.setPointer("converter",_converter);
  }

  void PWNDepthConverterSensor::deserialize(ObjectData& data, IdContext& context){
    BaseSensor::deserialize(data,context);
    data.getReference("converter").bind(_converter);
  }

  PWNDepthConvertedData::PWNDepthConvertedData(PWNDepthConverterSensor* sensor_,  int id, IdContext* context):
    PWNSensorData(id, context){
    _image = 0;
    _scale = 1;
    _sensor = sensor_;
  }
  
  void PWNDepthConvertedData::serialize(ObjectData& data, IdContext& context){
    PWNSensorData::serialize(data,context);
    data.setPointer("image", _image);
    data.setInt("scale", _scale);
  }

  void PWNDepthConvertedData::deserialize(ObjectData& data, IdContext& context){
    PWNSensorData::deserialize(data,context);
    data.getReference("image").bind(_image);
    _scale = data.getInt("scale");
  }


  BaseSensor* PWNDepthConvertedData::baseSensor() {return _sensor;}
  const BaseSensor* PWNDepthConvertedData::baseSensor() const {return _sensor;}


  BOSS_REGISTER_CLASS(PWNSensorData);
  BOSS_REGISTER_CLASS(PWNDepthConvertedData);
  BOSS_REGISTER_CLASS(PWNDepthConverterSensor);

}
