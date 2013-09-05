#ifndef _BOSS_ROBOT_CONFIGURATION_H_
#define _BOSS_ROBOT_CONFIGURATION_H_
#include "bframe.h"
#include "bsensor.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include <string>

namespace boss {
 class RobotConfiguration: public Serializable {
  public:
   RobotConfiguration(const std::string& name_="");
   const boss::StringSensorMap& sensorMap() const {return _sensorMap;}
   const boss::StringFrameMap&  frameMap() const {return _frameMap;}
   bool addSensor(BaseSensor* sensor_);
   bool addFrame(Frame* frame_);
   BaseSensor* sensor(const std::string topic);
   Frame* frame(const std::string name);
   inline const std::string& name() const {return _name;}
   inline void setName(const std::string& name_) {_name = name_;}
   inline bool isReady() const {return _isReady;}
   inline const std::string& baseFrameId() const { return _baseFrameId;}
   inline void setBaseFrameId(const std::string& baseFrameId_) {_baseFrameId = baseFrameId_;}
   virtual void serialize(ObjectData& data, IdContext& context);
   virtual void deserialize(ObjectData& data, IdContext& context);
   virtual void deserializeComplete();
   void serializeInternals(Serializer& ser);
 protected:
   bool _isReady;
   void isReadyUpdate();
   StringSensorMap _sensorMap;
   StringFrameMap _frameMap;
   std::string _baseFrameId;
   std::string _name;
 };


  
  RobotConfiguration* readLog(std::vector<BaseSensorData*>& sensorDatas, Deserializer& des);

} // end namespace
#endif

