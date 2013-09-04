#ifndef _BOSS_SENSOR_H_
#define _BOSS_SENSOR_H_
#include "bframe.h"
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include <string>
#include <deque>

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
 };

  class BaseSensorData: public Identifiable{
  public:
    BaseSensorData(int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context) {
      Identifiable::serialize(data,context);
      data.setDouble("timestamp",_timestamp);
      data.setString("topic",_topic);
      data.setPointer("robotFrame", _robotFrame);
    }
    virtual void deserialize(ObjectData& data, IdContext& context) {
      Identifiable::deserialize(data,context);
      _timestamp=data.getDouble("timestamp");
      _topic=data.getString("topic");
      data.getReference("robotFrame").bind(_robotFrame);
    }
    inline double timestamp() const {return _timestamp; }
    inline void setTimestamp(double timestamp_) { _timestamp=timestamp_;}
    inline const std::string& topic() const { return _topic; }
    inline void setTopic(const std::string topic_) {_topic = topic_; }
    inline Frame* robotFrame() {return _robotFrame;}
    inline const Frame* robotFrame() const {return _robotFrame;}
    inline void setRobotFrame(Frame* frame_) {_robotFrame=frame_;}
    virtual BaseSensor* baseSensor() {return 0;}
    virtual const BaseSensor* baseSensor() const {return 0;}
  protected:
    std::string _topic;
    double _timestamp;
    Frame* _robotFrame; // the frame where the  robot was when acquired the data
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
    virtual BaseSensor* baseSensor() {return sensor();}
    virtual const BaseSensor* baseSensor() const {return sensor();}
  protected:
    Sensor* _sensor;
  };
  

  typedef std::map<std::string, boss::Frame*> StringFrameMap;
  typedef std::map<std::string, boss::BaseSensor*> StringSensorMap;
  typedef std::deque<boss::Serializable*> SerializableQueue;

} // end namespace
#endif

