#ifndef _BOSS_SYNCHRONIZER_H_
#define _BOSS_SYNCHRONIZER_H_

#include <set>
#include <list>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "reference_frame.h"
#include "reference_frame_relation.h"
#include "sensor.h"
#include "stream_processor.h"

namespace boss_map {
  using namespace boss;

  struct SyncTopicInstance;

  struct SyncCondition{
    SyncCondition(SyncTopicInstance* m1, SyncTopicInstance*m2);
    virtual bool eval() = 0;
    virtual bool canEval();
    SyncTopicInstance* m1, *m2;
  };

  struct SyncTopicInstance{
    SyncTopicInstance(std::string topic);
    std::string topic;
    BaseSensorData* sensorData;
    std::set<SyncCondition*> syncConditions;
  };

  struct SyncTimeCondition : public SyncCondition{
    SyncTimeCondition(SyncTopicInstance* m1, SyncTopicInstance*m2, double dt);
    virtual bool eval();
    double dt;
  };


  struct SynchronizedSensorData: public BaseSensorData {
    SynchronizedSensorData(int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);

    template <class SensorDataType> 
    SensorDataType* sensorData(const std::string& topic_){
      size_t i = 0;
      for (; i<sensorDatas.size() && sensorDatas[i]->topic()!=topic_; i++);
      SensorDataType* s = 0;
      if (i<sensorDatas.size())
	s=dynamic_cast<SensorDataType*>(sensorDatas[i]);
      if (! s ){
	std::cerr << "type mismatch" << "topic: " << topic_ << "does not match the requested type" << std::endl;
      }
      return i<sensorDatas.size() ? dynamic_cast<SensorDataType*>(sensorDatas[i]) : 0;
    }

    std::vector<BaseSensorData*> sensorDatas;
  };

  struct SensorDataSynchronizer: public StreamProcessor{
    SensorDataSynchronizer();
    SyncTopicInstance* addSyncTopic(const std::string& topic);
    SyncTimeCondition* addSyncTimeCondition(const std::string& topic1, const std::string& topic2, double time);
    virtual void process(Serializable* s);
    SyncTopicInstance*  syncTopic(std::string topic);
    ~SensorDataSynchronizer();
    inline const std::string& topic() const {return _topic;}
    inline void setTopic (const std::string& topic_) {_topic = topic_;}
    
  protected:
    std::string _topic;
    void computeDependancies(std::set<SyncCondition*> & conditions, 
			     std::set<SyncTopicInstance*>& dependancies,
			     SyncTopicInstance* instance);
    bool addSyncTopic(SyncTopicInstance* st);
    bool addSyncCondition(SyncCondition* cond);  
    bool addSensorData(BaseSensorData* data);


    std::map<std::string, SyncTopicInstance*> syncTopics;
    std::set<SyncCondition*> syncConditions;
  };
  
}
#endif

