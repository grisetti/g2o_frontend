#include "stream_processor.h"
#include <stdexcept>
#include <iostream>
#include "g2o_frontend/boss/object_data.h"

namespace boss_map{
  using namespace boss;
  using namespace std;

  StreamProcessor::OutputHandler::OutputHandler(StreamProcessor* processor_, int id, boss::IdContext* context):
    Identifiable(id, context) {
    _processor = _bp = 0;
    setStreamProcessor(processor_);
  }


  void StreamProcessor::OutputHandler::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data, context);
    data.setPointer("source", _processor);
  }
  
  void StreamProcessor::OutputHandler::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data, context);
    _bp = 0;
    data.getReference("source").bind(_bp);
  }

  void StreamProcessor::OutputHandler::deserializeComplete(){
    setStreamProcessor(_bp);
  }


  void StreamProcessor::OutputHandler::setStreamProcessor(StreamProcessor* p) {
    if (p == _processor)
      return;
    if (_processor)
      _processor->removeHandler(this);
   
    _processor = p;
    if (! _processor)
      return;

    bool result = _processor->addHandler(this);
    if (!result){
      throw std::runtime_error ("cannot add handler to the processor");
    }
  }
      
  StreamProcessor::OutputHandler::~OutputHandler(){
    bool result = _processor->removeHandler(this);
    if (!result){
      throw std::runtime_error ("cannot erase handler to the processor");
    }
  }

  StreamProcessor::WriterOutputHandler::WriterOutputHandler(StreamProcessor* processor_, boss::Serializer* ser_, int id, boss::IdContext* context):
    StreamProcessor::OutputHandler(processor_, id, context){
    _serializer = ser_;
  }
  
  void StreamProcessor::WriterOutputHandler::put(boss::Serializable* s){
    _serializer->writeObject(*s);
  }
  
  StreamProcessor::EnqueuerOutputHandler::EnqueuerOutputHandler(StreamProcessor* processor_, 
								SerializableList* serializables_,
								int id, boss::IdContext* context):
    StreamProcessor::OutputHandler(processor_, id, context){
    _serializables = serializables_;
  }

  void StreamProcessor::EnqueuerOutputHandler::put(boss::Serializable* s){
    _serializables->push_back(s);
  }

  StreamProcessor::PropagatorOutputHandler::PropagatorOutputHandler(StreamProcessor* processor_, StreamProcessor* destinationProcessor_, int id, boss::IdContext* context): StreamProcessor::OutputHandler(processor_, id, context){
    _destinationProcessor = destinationProcessor_;
  }

  void StreamProcessor::PropagatorOutputHandler::serialize(ObjectData& data, IdContext& context){
    StreamProcessor::OutputHandler::serialize(data, context);
    data.setPointer("sink", _destinationProcessor);
  }
  
  void StreamProcessor::PropagatorOutputHandler::deserialize(ObjectData& data, IdContext& context){
    StreamProcessor::OutputHandler::deserialize(data, context);
    data.getReference("sink").bind(_destinationProcessor);
  }
  
  void StreamProcessor::PropagatorOutputHandler::put(boss::Serializable* s){
    _destinationProcessor->process(s);
  }

  void StreamProcessor::put(boss::Serializable* s){
    for(std::list<OutputHandler*>::iterator it = _handlers.begin(); it!=_handlers.end(); it++){
      OutputHandler* handler = *it;
      handler->put(s);
    }
  }

  bool StreamProcessor::addHandler(OutputHandler* handler_){
    std::list<OutputHandler*>::iterator it = findHandler(handler_);
    if (it == _handlers.end()){
      _handlers.push_back(handler_);
      return true;
    }
    return false;
  }

  bool StreamProcessor::removeHandler(OutputHandler* handler_) {
    std::list<OutputHandler*>::iterator it = findHandler(handler_);
    if (it != _handlers.end()){
      _handlers.erase(it);
      return true;
    }
    return false;
  }

  std::list<StreamProcessor::OutputHandler*>::iterator StreamProcessor::findHandler(OutputHandler* handler_){
    for(std::list<OutputHandler*>::iterator it = _handlers.begin(); it!=_handlers.end(); it++){
      OutputHandler* handler = *it;
      if (handler == handler_)
	return it;
    }
    return _handlers.end();
  }

  StreamProcessor::StreamProcessor(int id, boss::IdContext* context):
    Identifiable(id, context){
    _name = "unknown";
    _robotConfiguration = 0;
  }

  
  StreamProcessor::~StreamProcessor(){
    std::list<OutputHandler*> l = _handlers;
    for(std::list<OutputHandler*>::iterator it = l.begin(); it!=l.end(); it++){
      OutputHandler* handler = *it;
      delete handler;
    }
  }

  RobotConfiguration* StreamProcessor::robotConfiguration() {
    return _robotConfiguration;
  }
  
  void StreamProcessor::setRobotConfiguration(RobotConfiguration* conf){
    _robotConfiguration = conf;
  }


  void StreamProcessor::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    data.setString("name", _name);
  }
  
  void StreamProcessor::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    _name = data.getString("name");
  }




  StreamProcessorGroup::StreamProcessorGroup(int id, boss::IdContext* context):
    StreamProcessor(id,context){
    firstNode = 0;
    lastNode = 0;
  }

  void StreamProcessorGroup::setRobotConfiguration(RobotConfiguration* conf){
    StreamProcessor::setRobotConfiguration(conf);
    for (std::map<std::string, StreamProcessor*>::iterator it=streamProcessors.begin(); it!=streamProcessors.end(); it++)
      it->second->setRobotConfiguration(conf);
  }

  void StreamProcessorGroup::serialize(boss::ObjectData& data, boss::IdContext& context){
    StreamProcessor::serialize(data,context);
    data.setPointer("firstNode", firstNode);
    data.setPointer("lastNode", lastNode);
    ArrayData* objectArray = new ArrayData();
    for (size_t i =0; i<objects.size(); i++){
      objectArray->add(new PointerData(objects[i]));
    }
    data.setField("objects", objectArray);
  }

  void StreamProcessorGroup::deserialize(boss::ObjectData& data, boss::IdContext& context){
    StreamProcessor::deserialize(data,context);
    objects.clear();
    streamProcessors.clear();
    firstNode = 0;
    lastNode = 0;
    data.getReference("firstNode").bind(firstNode);
    data.getReference("lastNode").bind(lastNode);
    ArrayData& objectArray = data.getField("objects")->getArray();
    for (size_t i =0; i<objectArray.size(); i++){
      Identifiable* ident=objectArray[i].getPointer();
      objects.push_back(ident);
      StreamProcessor* proc=dynamic_cast<StreamProcessor*>(ident);
      if (proc) {
	streamProcessors.insert(make_pair(proc->name(), proc));
      }
    }
  }
  
  void StreamProcessorGroup::process(Serializable* s) {
    if (firstNode)
      firstNode->process(s);
  }

  StreamProcessor* StreamProcessorGroup::byName(const std::string& n) {
    std::map<std::string, StreamProcessor*>::iterator it = streamProcessors.find(n);
    if (it!=streamProcessors.end())
      return it->second;
    return 0;
  }

  StreamProcessor* loadProcessor(const std::string& name, boss::Deserializer& des, std::list<Serializable*>& objects){
    Serializable* o;
    while( (o = des.readObject()) ) {
      objects.push_back(o);
      StreamProcessor* proc=dynamic_cast<StreamProcessor*>(o);
      if (proc &&  proc->name() == name )
	return proc;
    }
    return 0;
  }

  typedef StreamProcessor::EnqueuerOutputHandler StreamProcessor_EnqueuerOutputHandler;
  typedef StreamProcessor::WriterOutputHandler StreamProcessor_WriterOutputHandler;
  typedef StreamProcessor::PropagatorOutputHandler StreamProcessor_PropagatorOutputHandler;

  BOSS_REGISTER_CLASS(StreamProcessor_EnqueuerOutputHandler);
  BOSS_REGISTER_CLASS(StreamProcessor_WriterOutputHandler);
  BOSS_REGISTER_CLASS(StreamProcessor_PropagatorOutputHandler);
  BOSS_REGISTER_CLASS(StreamProcessorGroup);

}
