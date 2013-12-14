#pragma once
#include <list>
#include "g2o_frontend/boss/serializer.h"

namespace boss_map {
  typedef std::list<boss::Serializable*> SerializableList;

  class StreamProcessor{
  public:
    class OutputHandler{
    public:
      OutputHandler(StreamProcessor* processor_);
      inline StreamProcessor* streamProcessor() { return _processor; }
      virtual void put(boss::Serializable* s)=0;
      virtual ~OutputHandler();
    protected:
      StreamProcessor* _processor;
    };

    class WriterOutputHandler: public OutputHandler {
    public:
      WriterOutputHandler(StreamProcessor* processor_, boss::Serializer* ser_);
      inline boss::Serializer* serializer() {return _serializer;}
      virtual void put(boss::Serializable* s);
    protected:
      boss::Serializer* _serializer;
    };

    class EnqueuerOutputHandler: public OutputHandler {
    public:
      EnqueuerOutputHandler(StreamProcessor* processor_, SerializableList* serializables_);
      inline SerializableList* serializer() {return _serializables;}
      virtual void put(boss::Serializable* s);
    protected:
      SerializableList * _serializables;
    };

    class PropagatorOutputHandler: public OutputHandler {
    public:
      PropagatorOutputHandler(StreamProcessor* processor_, StreamProcessor* destinationProcessor_);
      inline StreamProcessor* destinationProcessor() {return _destinationProcessor;};
      virtual void put(boss::Serializable* s);
    protected:
      StreamProcessor* _destinationProcessor;
    };

    friend class OutputHandler;

    virtual ~StreamProcessor();
    virtual void process(boss::Serializable*s) = 0;
    virtual void put(boss::Serializable* s);
  protected:
    bool addHandler(OutputHandler* handler_);
    bool removeHandler(OutputHandler* handler_);
    std::list<OutputHandler*>::iterator findHandler(OutputHandler* handler_);
    std::list<OutputHandler*> _handlers;
  };
}
