#include "stream_processor.h"
#include <stdexcept>
#include <iostream>

namespace boss_map{
  using namespace std;

  StreamProcessor::OutputHandler::OutputHandler(StreamProcessor* processor_) {
    _processor = processor_;
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

  StreamProcessor::WriterOutputHandler::WriterOutputHandler(StreamProcessor* processor_, boss::Serializer* ser_):
    StreamProcessor::OutputHandler(processor_){
    _serializer = ser_;
  }
  
  void StreamProcessor::WriterOutputHandler::put(boss::Serializable* s){
    _serializer->writeObject(*s);
  }
  
  StreamProcessor::EnqueuerOutputHandler::EnqueuerOutputHandler(StreamProcessor* processor_, 
								SerializableList* serializables_):
    StreamProcessor::OutputHandler(processor_){
    _serializables = serializables_;
  }

  void StreamProcessor::EnqueuerOutputHandler::put(boss::Serializable* s){
    _serializables->push_back(s);
  }

  StreamProcessor::PropagatorOutputHandler::PropagatorOutputHandler(StreamProcessor* processor_, StreamProcessor* destinationProcessor_): StreamProcessor::OutputHandler(processor_){
    _destinationProcessor = destinationProcessor_;
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

  
  StreamProcessor::~StreamProcessor(){
    std::list<OutputHandler*> l = _handlers;
    for(std::list<OutputHandler*>::iterator it = l.begin(); it!=l.end(); it++){
      OutputHandler* handler = *it;
      delete handler;
    }
  }

}
