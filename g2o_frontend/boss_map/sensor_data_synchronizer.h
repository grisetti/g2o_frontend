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
    std::vector<BaseSensorData*> sensorDatas;
  };

  struct Synchronizer: public StreamProcessor{

    enum DroppedDataPolicy {KeepData,DeleteData};
    enum DroppedReferenceFramePolicy {KeepReferenceFrame,DeleteReferenceFrame};

    Synchronizer();
    SyncTopicInstance* addSyncTopic(const std::string& topic);
    SyncTimeCondition* addSyncTimeCondition(const std::string& topic1, const std::string& topic2, double time);
    virtual void process(Serializable* s);
    SyncTopicInstance*  syncTopic(std::string topic);
    void syncDone();
    ~Synchronizer();
  
  protected:
    void computeDependancies(std::set<SyncCondition*> & conditions, 
			     std::set<SyncTopicInstance*>& dependancies,
			     SyncTopicInstance* instance);
    bool addSyncTopic(SyncTopicInstance* st);
    bool addSyncCondition(SyncCondition* cond);  
    bool addSensorData(BaseSensorData* data);


    std::map<std::string, SyncTopicInstance*> syncTopics;
    std::set<SyncCondition*> syncConditions;
    DroppedReferenceFramePolicy framePolcy;
    DroppedDataPolicy  dataPolicy;
    std::deque<BaseSensorData*> _syncDatas;
  };

  
}
#endif

