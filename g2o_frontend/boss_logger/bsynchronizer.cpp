#include "bsynchronizer.h"
#include <stdexcept>
namespace boss {

  using namespace std;

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


  Synchronizer::Synchronizer(){
    dataPolicy = DeleteData;
    framePolcy = DeleteReferenceFrame;
  }
  
  SyncTopicInstance*  Synchronizer::syncTopic(std::string topic) {
    std::map<std::string, SyncTopicInstance*>::iterator it  = syncTopics.find(topic);
    if (it == syncTopics.end())
      return 0;
    return it->second;
  }

  bool Synchronizer::addSyncTopic(SyncTopicInstance* st) {
    SyncTopicInstance* previous = syncTopic(st->topic);
    if (! previous){
      syncTopics.insert(make_pair(st->topic, st));
      //cerr << "added sync topic" << st->topic << endl;
      return true;
    }
    return false;
  }

  bool Synchronizer::addSyncCondition(SyncCondition* cond){
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
  
  bool Synchronizer::addSensorData(BaseSensorData* data){
    SyncTopicInstance* st = syncTopic(data->topic());
    if (! st) {
      // the data is not synchronized, we just pass it through;
      for (size_t i =0; i<outputHandlers.size(); i++){
	outputHandlers[i]->put(data);
      }
      for (size_t i =0; i<outputHandlers.size(); i++){
	outputHandlers[i]->syncDone();
      }
      return true;
    }

    // replace the old copy of the data
    if (st->sensorData){
      if (st->sensorData->robotReferenceFrame() && framePolcy == DeleteReferenceFrame)
	delete st->sensorData->robotReferenceFrame();
      if (st->sensorData && dataPolicy == DeleteData)
	delete st->sensorData;
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
      for (std::set<SyncTopicInstance*>::iterator it = dependancies.begin(); it!=dependancies.end(); it++) {
	for (size_t i =0; i<outputHandlers.size(); i++){
	  BaseSensorData* other = (*it)->sensorData;
	    outputHandlers[i]->put(other);
	}
      }
      for (size_t i =0; i<outputHandlers.size(); i++){
	outputHandlers[i]->syncDone();
      }
      for (std::set<SyncTopicInstance*>::iterator it = dependancies.begin(); it!=dependancies.end(); it++) {
	(*it)->sensorData=0;
      }	
      return true;
    }
    
    return false;
      
  }

  void Synchronizer::addOutputHandler(OutputHandler* handler){
    outputHandlers.push_back(handler);
    handler->setSynchronizer(this);
  }

  SyncTopicInstance* Synchronizer::addSyncTopic(const std::string& topic){
    std::map<std::string, SyncTopicInstance*>::iterator it  = syncTopics.find(topic);
    if (it!=syncTopics.end())
      return it->second;
    SyncTopicInstance* instance = new SyncTopicInstance(topic);
    syncTopics.insert(make_pair(topic,instance));
    //cerr << "added sync topic" << topic << endl;
    return instance;
  }

  SyncTimeCondition* Synchronizer::addSyncTimeCondition(const std::string& topic1, const std::string& topic2, double time){
    SyncTopicInstance* m1 = addSyncTopic(topic1);
    SyncTopicInstance* m2 = addSyncTopic(topic2);
    SyncTimeCondition* cond = new SyncTimeCondition(m1, m2, time);
    addSyncCondition(cond);
    return cond;
  }

  void Synchronizer::computeDependancies(std::set<SyncCondition*> & conditions, 
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


  
  Synchronizer::OutputHandler::OutputHandler() {
    synchronizer = 0;
  }

  void Synchronizer::OutputHandler::put(BaseSensorData* s) {
    _syncDatas.push_back(s);
  }

  void Synchronizer::OutputHandler::syncDone() {
    syncDoneImpl();
    _syncDatas.clear();
  }

  void Synchronizer::Reframer::syncDoneImpl(){
    if (_syncDatas.empty())
      return;
    ReferenceFrame * f=_syncDatas.front()->robotReferenceFrame();
    while (! _syncDatas.empty()){
      BaseSensorData* data = _syncDatas.front();
      _syncDatas.pop_front();
      if(data->robotReferenceFrame() != f){
	if (synchronizer->framePolcy == Synchronizer::DeleteReferenceFrame)
	  delete data->robotReferenceFrame();
	data->setRobotReferenceFrame(f);
      }
    }
  }

  Synchronizer::Writer::Writer(Serializer* ser) {
    this->ser = ser;
  }
  void Synchronizer::Writer::syncDoneImpl(){
    if (_syncDatas.empty())
      return;
    std::set<ReferenceFrame*> frames;
    while (! _syncDatas.empty()){
      BaseSensorData* data = _syncDatas.front();
      ReferenceFrame * f=data->robotReferenceFrame();
      if (frames.find(f)==frames.end()){
	ser->writeObject(*f);
	frames.insert(f);
      }
      _syncDatas.pop_front();
      ser->writeObject(*data);
    }
  }

  void Synchronizer::Deleter::syncDoneImpl(){
    std::set<ReferenceFrame*> frames;
    while (! _syncDatas.empty()){
      BaseSensorData* data = _syncDatas.front();
      frames.insert(data->robotReferenceFrame());
      _syncDatas.pop_front();
      delete data;
    }
    for (std::set<ReferenceFrame*>::iterator it =  frames.begin(); it!= frames.end(); it++){
      ReferenceFrame* f = *it;
      delete f;
    }
      
  }

}
