#ifndef _BOSS_SENSOR_H_
#define _BOSS_SENSOR_H_
#include "bframe.h"
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include <string>

namespace boss {
 class BaseSensor: public Identifiable{
  public:
   BaseSensor(int id=-1, IdContext* context = 0);
   virtual void serialize(ObjectData& data, IdContext& context);
   virtual void deserialize(ObjectData& data, IdContext& context);
   virtual void deserializeComplete();
   inline const std::string& topic() const { return _topic; }
   inline void setTopic(const std::string topic_) {_topic = topic_; }
   inline Frame* frame() { return _frame; }   
   inline const Frame* frame() const { return _frame;}
   inline void setFrame(Frame* frame_) { _frame = frame_;}
 protected:
   Frame* _frame;
   std::string _topic;
 private:
   Identifiable* _tempFrame;
 };

  class BaseSensorData: public Identifiable{
  public:
    BaseSensorData(int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context) {
      Identifiable::serialize(data,context);
      data.setDouble("timestamp",_timestamp);
    }
    virtual void deserialize(ObjectData& data, IdContext& context) {
      Identifiable::deserialize(data,context);
      _timestamp=data.getDouble("timestamp");
    }
    inline double timestamp() const {return _timestamp; }
    inline void setTimestamp(double timestamp_) { _timestamp=timestamp_;}
  protected:
    double _timestamp;
  };


  template <typename SensorType>
  class SensorData: public BaseSensorData{
  public:
    typedef SensorType Sensor;
    SensorData(int id=-1, IdContext* context = 0): BaseSensorData(id,context) {_sensor = 0;}
    virtual void serialize(ObjectData& data, IdContext& context) {
      BaseSensorData::serialize(data,context);
      data.setPointer("sensor", _sensor);
    }
    virtual void deserialize(ObjectData& data, IdContext& context) {
      BaseSensorData::deserialize(data,context);
      _sensor=static_cast<Sensor*>(data.getPointer("sensor"));
    }

    inline const Sensor* sensor() const {return _sensor;}
    inline Sensor* sensor() {return _sensor;}
    inline void setSensor(Sensor* sensor_) {_sensor = sensor_;}
  protected:
    Sensor* _sensor;
  };

} // end namespace
#endif
