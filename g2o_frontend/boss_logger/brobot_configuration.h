#ifndef _BOSS_ROBOT_CONFIGURATION_H_
#define _BOSS_ROBOT_CONFIGURATION_H_
#include "bframe.h"
#include "bsensor.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include <string>

namespace boss_logger {
  using namespace boss;
 class RobotConfiguration: public Serializable {
  public:
   RobotConfiguration(const std::string& name_="");
   const StringSensorMap& sensorMap() const {return _sensorMap;}
   const StringReferenceFrameMap&  frameMap() const {return _frameMap;}
   bool addSensor(BaseSensor* sensor_);
   bool addReferenceFrame(ReferenceFrame* frame_);
   BaseSensor* sensor(const std::string topic);
   ReferenceFrame* frame(const std::string name);
   inline ReferenceFrame* baseFrame() { return frame(_baseReferenceFrameId); }
   inline const std::string& name() const {return _name;}
   inline void setName(const std::string& name_) {_name = name_;}
   inline bool isReady() const {return _isReady;}
   inline const std::string& baseReferenceFrameId() const { return _baseReferenceFrameId;}
   inline void setBaseReferenceFrameId(const std::string& baseReferenceFrameId_) {_baseReferenceFrameId = baseReferenceFrameId_;}
   virtual void serialize(ObjectData& data, IdContext& context);
   virtual void deserialize(ObjectData& data, IdContext& context);
   virtual void deserializeComplete();
   void serializeInternals(Serializer& ser);
   Eigen::Isometry3d sensorOffset(const BaseSensor* sensor) const;
 protected:
   bool _isReady;
   void isReadyUpdate();
   StringSensorMap _sensorMap;
   StringReferenceFrameMap _frameMap;
   std::string _baseReferenceFrameId;
   std::string _name;
 };


  
  RobotConfiguration* readLog(std::vector<BaseSensorData*>& sensorDatas, Deserializer& des);

} // end namespace
#endif

