#include "sensor_data_synchronizer.h"
#include <stdexcept>

namespace boss_map {  
  using namespace std;
  using namespace boss;

  SynchronizedSensorData::SynchronizedSensorData(int id, IdContext* context): BaseSensorData(id, context) {
  }
  
  void SynchronizedSensorData::serialize(ObjectData& data, IdContext& context){
    BaseSensorData::serialize(data,context);
    ArrayData* dataArray = new ArrayData();
    for (size_t i =0; i<sensorDatas.size(); i++){
      BaseSensorData* s = sensorDatas[i];
      dataArray->add(new PointerData(s));
    }
    data.setField("sensorDatas", dataArray);
  }
    
  void SynchronizedSensorData::deserialize(ObjectData& data, IdContext& context){
    BaseSensorData::deserialize(data,context);
    ArrayData& dataArray = data.getField("sensorDatas")->getArray();
    sensorDatas.resize(dataArray.size());
    for (size_t i =0; i<dataArray.size(); i++){
      Identifiable* id = dataArray[i].getPointer();
      sensorDatas[i] = dynamic_cast<BaseSensorData*>(id);
    }
  }


  SyncCondition::SyncCondition(SyncTopicInstance* m1, SyncTopicInstance*m2){
    this->m1  = m1;
    this->m2  = m2;
  }


  bool SyncCondition::canEval(){
    bool ret =  m1 && m2 && m1->sensorData && m2->sensorData;
    return ret;
  }

  SyncTopicInstance::SyncTopicInstance(std::string topic){
    this->topic = topic;    sensorData = 0;
  }


  SyncTimeCondition::SyncTimeCondition(SyncTopicInstance* m1, SyncTopicInstance*m2, double dt): SyncCondition(m1, m2){
    this->m1 = m1;
    this->m2 = m2;
    this->dt = dt;
  }

  bool SyncTimeCondition::eval() {
    if (! canEval())
      return true;
    bool ret = fabs(m1->sensorData->timestamp()-m2->sensorData->timestamp())<dt;
    return ret;

  }

  SensorDataSynchronizer::SensorDataSynchronizer(){
  }
  
  SyncTopicInstance*  SensorDataSynchronizer::syncTopic(std::string topic) {
    std::map<std::string, SyncTopicInstance*>::iterator it  = syncTopics.find(topic);
    if (it == syncTopics.end())
      return 0;
    return it->second;
  }

  bool SensorDataSynchronizer::addSyncTopic(SyncTopicInstance* st) {
    SyncTopicInstance* previous = syncTopic(st->topic);
    if (! previous){
      syncTopics.insert(make_pair(st->topic, st));
      //cerr << "added sync topic" << st->topic << endl;
      return true;
    }
    return false;
  }

  bool SensorDataSynchronizer::addSyncCondition(SyncCondition* cond){
    SyncTopicInstance* other1=syncTopic(cond->m1->topic);
    SyncTopicInstance* other2=syncTopic(cond->m2->topic);
    if (other1!=cond->m1 || other2!=cond->m2) {
      cerr << "failure in adding condition" << endl;
      cerr << other1->topic << " != " << cond->m1->topic << endl;
      cerr << other2->topic << " != " << cond->m2->topic << endl;
      return false;
    }
    syncConditions.insert(cond);
    cond->m1->syncConditions.insert(cond);
    cond->m2->syncConditions.insert(cond);
    return true;
  }
  
  bool SensorDataSynchronizer::addSensorData(BaseSensorData* data){
    SyncTopicInstance* st = syncTopic(data->topic());
    if (! st) {
      return true;
    }

    st->sensorData = data;

    // check if the dependancies are satisfied
    bool canEval = true;
    bool allSatisfied = true;
    std::set<SyncCondition*> conditions;
    std::set<SyncTopicInstance*> dependancies;
    computeDependancies(conditions, dependancies, st);
    for (std::set<SyncCondition*>::iterator it = conditions.begin(); it!=conditions.end(); it++){
      SyncCondition* cond = *it;
      canEval &= cond->canEval();
      if(cond->canEval())
	allSatisfied &= cond->eval();
    }
    if (! canEval){
      cerr << 'x';
      return false;
    }
    if (allSatisfied) {
      cerr << 'F';
      SynchronizedSensorData* ssd = new SynchronizedSensorData();
      ssd->setTopic(_topic);
      for (std::set<SyncTopicInstance*>::iterator it = dependancies.begin(); it!=dependancies.end(); it++) {
	ssd->sensorDatas.push_back((*it)->sensorData);
	(*it)->sensorData = 0;
      }
      ssd->setRobotReferenceFrame(data->robotReferenceFrame());
      ssd->setTimestamp(data->timestamp());
      put(ssd);
      return true;
    }
    return false;
  }


  SyncTopicInstance* SensorDataSynchronizer::addSyncTopic(const std::string& topic){
    std::map<std::string, SyncTopicInstance*>::iterator it  = syncTopics.find(topic);
    if (it!=syncTopics.end())
      return it->second;
    SyncTopicInstance* instance = new SyncTopicInstance(topic);
    syncTopics.insert(make_pair(topic,instance));
    //cerr << "added sync topic" << topic << endl;
    return instance;
  }

  SyncTimeCondition* SensorDataSynchronizer::addSyncTimeCondition(const std::string& topic1, const std::string& topic2, double time){
    SyncTopicInstance* m1 = addSyncTopic(topic1);
    SyncTopicInstance* m2 = addSyncTopic(topic2);
    SyncTimeCondition* cond = new SyncTimeCondition(m1, m2, time);
    addSyncCondition(cond);
    return cond;
  }

  void SensorDataSynchronizer::process(Serializable*s){
    put(s);
    BaseSensorData* sdata = dynamic_cast<BaseSensorData*>(s);
    if (sdata)
      addSensorData(sdata);
  }

  void SensorDataSynchronizer::computeDependancies(std::set<SyncCondition*> & conditions, 
					 std::set<SyncTopicInstance*>& dependancies,
					 SyncTopicInstance* instance){
    dependancies.clear();
    dependancies.insert(instance);
    std::deque<SyncTopicInstance*> deque;
    deque.push_back(instance);
    while(! deque.empty()){
      SyncTopicInstance* current  = deque.front();
      deque.pop_front();
      for ( std::set<SyncCondition*>::iterator it=current->syncConditions.begin(); 
	    it != current->syncConditions.end(); it++) {
	SyncTopicInstance* t1 = (*it)->m1;
        SyncTopicInstance* t2 = (*it)->m2;
	if (dependancies.find(t1)==dependancies.end()){
	  deque.push_back(t1);
	  dependancies.insert(t1);
	}
	if (dependancies.find(t2)==dependancies.end()){
	  deque.push_back(t2);
	  dependancies.insert(t2);
	}
      }
    }
    conditions.clear();
    for (std::set<SyncTopicInstance*>::iterator it=dependancies.begin(); it!=dependancies.end(); it++){
      for(std::set<SyncCondition*>::iterator cit=(*it)->syncConditions.begin(); cit!=(*it)->syncConditions.end(); cit++)
	conditions.insert(*cit);
    }
  }

  SensorDataSynchronizer::~SensorDataSynchronizer(){
    for (std::map<std::string, SyncTopicInstance*>::iterator it = syncTopics.begin(); it!=syncTopics.end(); it++){
      delete it->second;
    }
    for (std::set<SyncCondition*>::iterator it=syncConditions.begin(); it!=syncConditions.end(); it++){
      delete *it;
    }

  }


  BOSS_REGISTER_CLASS(SynchronizedSensorData);
}
