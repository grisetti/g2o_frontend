#include "brobot_configuration.h"
#include <stdexcept>
#include <iostream>

namespace boss {
  using namespace std;

  RobotConfiguration::RobotConfiguration::RobotConfiguration(const std::string& name_) {
    _name = name_;
    _baseFrameId = "/base_link";
    _isReady = true;
  }

  bool RobotConfiguration::addSensor(BaseSensor* sensor_) {
    BaseSensor* s=sensor(sensor_->topic());
    if (s)
      return false;
    _sensorMap.insert(make_pair(sensor_->topic(),sensor_));
    isReadyUpdate();
    return true;
  }

  bool RobotConfiguration::addFrame(Frame* frame_){
    Frame* f=frame(frame_->name());
    if (f)
      return false;
    _frameMap.insert(make_pair(frame_->name(),frame_));
    isReadyUpdate();
    return true;
  }

  BaseSensor* RobotConfiguration::sensor(const std::string topic) {
    std::map<std::string, BaseSensor*>::iterator it=_sensorMap.find(topic);
    if(it==_sensorMap.end())
      return 0;
    return it->second;
  }

  Frame* RobotConfiguration::frame(const std::string name){
    std::map<std::string, Frame*>::iterator it=_frameMap.find(name);
    if(it==_frameMap.end())
      return 0;
    return it->second;
  }

  void RobotConfiguration::isReadyUpdate(){
    _isReady = false;
    Frame * baseFrame = frame(_baseFrameId);
    if (! baseFrame)
      return;
    // for each sensor, check if you can determine the transformation to the base link
    for (std::map<std::string, BaseSensor*>::iterator it = _sensorMap.begin(); it!=_sensorMap.end(); it++){
      BaseSensor* s = it->second;
      if (! s) 
	return;
      Frame *  f = s->frame();
      if (! f)
	return;
      if (! f->canTransformTo(baseFrame))
	return;
    }
    _isReady = true;
  }


  void RobotConfiguration::serialize(ObjectData& data, IdContext& /*context*/) {
    /*Serializable::serialize(data,context);*/
    data.setString("name", _name);
    data.setString("baseFrameId", _baseFrameId);
    ArrayData* frameArray = new ArrayData;
    for (std::map<std::string, Frame*>::iterator it = _frameMap.begin(); it!=_frameMap.end(); it++){
      Frame* frame=it->second;
      frameArray->add(new PointerData(frame));
      cerr << "adding frame:" << it->second->name() << endl;
    }
    data.setField("frames", frameArray);
    ArrayData* sensorArray = new ArrayData;
    for (std::map<std::string, BaseSensor*>::iterator it = _sensorMap.begin(); it!=_sensorMap.end(); it++){
      BaseSensor* sensor=it->second;
      sensorArray->add(new PointerData(sensor));
      cerr << "adding sensor:" << it->second->topic() << endl;
    }
    data.setField("sensors", sensorArray);
  }
  
  void RobotConfiguration::deserialize(ObjectData& data, IdContext& /*context*/) {
    /*Serializable::deserialize(data,context);*/
    _name = data.getString("name");
    _baseFrameId = data.getString("baseFrameId");
    ArrayData& frameArray=data.getField("frames")->getArray();
    for (size_t i =0; i< frameArray.size(); i++){
      ValueData& v = frameArray[i];
      Identifiable* id = v.getPointer();
      Frame* f = dynamic_cast<Frame*>(id);
      if (f){
	addFrame(f);
      }
    }
    ArrayData& sensorArray=data.getField("sensors")->getArray();
    for (size_t i =0; i< sensorArray.size(); i++){
      ValueData& v = sensorArray[i];
      Identifiable* id = v.getPointer();
      BaseSensor* s = dynamic_cast<BaseSensor*>(id);
      if (s){
	cerr << "Adding Sensor: " << id->className() << endl;
	addSensor(s);
      }
    }
    isReadyUpdate();
  }
  
  void RobotConfiguration::deserializeComplete(){
    isReadyUpdate();
  }
  
  void RobotConfiguration::serializeInternals(Serializer& confSer){
    for(StringFrameMap::iterator it = _frameMap.begin(); it!=_frameMap.end(); it++){
      Frame* f = it->second;
      confSer.writeObject(*f);
    }

    for(StringSensorMap::iterator it = _sensorMap.begin(); it!=_sensorMap.end(); it++){
      BaseSensor* s = it->second;
      confSer.writeObject(*s);
    }
  }




  RobotConfiguration* readLog(std::vector<BaseSensorData*>& sensorDatas, Deserializer& des){
    
    RobotConfiguration* conf = 0;
    Serializable *o;
    int numObjects=0;
    while( (! conf && (o=des.readObject())) ){
      numObjects++;
      if (! conf) {
	conf = dynamic_cast<RobotConfiguration*>(o);
	if (conf) {
	  cerr << "got config" << endl;
	  if (! conf->isReady()){
	    cerr << "conf failure" << endl;
	    return 0;
	  }
	}
	continue;
      }
    }

    if (!conf) {
      cerr << "unable to read robot configuration, aborting" << endl;
      return 0;
    }

    while( (o=des.readObject()) ){
      BaseSensorData* sensorData=dynamic_cast<BaseSensorData*>(o);
      if(sensorData)
	sensorDatas.push_back(sensorData);
    }
    return conf;
  }

  BOSS_REGISTER_CLASS(RobotConfiguration);
}
