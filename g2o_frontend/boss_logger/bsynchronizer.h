#ifndef _BOSS_SYNCHRONIZER_H_
#define _BOSS_SYNCHRONIZER_H_

#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"
#include "bimagesensor.h"
#include "blasersensor.h"
#include "bimusensor.h"

namespace boss_logger {
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


  struct Synchronizer{
    struct OutputHandler{
      friend struct Synchronizer;
      OutputHandler();
      void put(BaseSensorData* s);
      virtual void syncDoneImpl()=0;
      void syncDone();
    protected:
      inline void setSynchronizer(Synchronizer* sync) {synchronizer = sync;}
      Synchronizer* synchronizer;
      std::deque<BaseSensorData*> _syncDatas;
    };

    struct Reframer: public OutputHandler{
      virtual void syncDoneImpl();
    };

    struct Writer: public OutputHandler{
      Writer(Serializer* ser);
      virtual void syncDoneImpl();
      Serializer* ser;
    };

    struct Deleter: public OutputHandler{
      virtual void syncDoneImpl();
    };

    enum DroppedDataPolicy {KeepData,DeleteData};
    enum DroppedReferenceFramePolicy {KeepReferenceFrame,DeleteReferenceFrame};

    Synchronizer();
    SyncTopicInstance* addSyncTopic(const std::string& topic);
    SyncTimeCondition* addSyncTimeCondition(const std::string& topic1, const std::string& topic2, double time);
    bool addSensorData(BaseSensorData* data);
    void addOutputHandler(OutputHandler* handler);
    SyncTopicInstance*  syncTopic(std::string topic);
  protected:
    void computeDependancies(std::set<SyncCondition*> & conditions, 
			     std::set<SyncTopicInstance*>& dependancies,
			     SyncTopicInstance* instance);
    bool addSyncTopic(SyncTopicInstance* st);
    bool addSyncCondition(SyncCondition* cond);  


    std::map<std::string, SyncTopicInstance*> syncTopics;
    std::set<SyncCondition*> syncConditions;
    DroppedReferenceFramePolicy framePolcy;
    DroppedDataPolicy  dataPolicy;
    std::vector<OutputHandler*> outputHandlers;
  };


  
}
#endif

