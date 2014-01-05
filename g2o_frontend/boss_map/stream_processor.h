#pragma once
#include <list>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/identifiable.h"

namespace boss_map {
  typedef std::list<boss::Serializable*> SerializableList;

  class StreamProcessor : public boss::Identifiable {
  public:
    class OutputHandler : public boss::Identifiable {
    public:
      OutputHandler(StreamProcessor* processor_=0, int id=-1, boss::IdContext* context = 0);
      virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
      virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
      virtual void deserializeComplete();

      inline StreamProcessor* streamProcessor() { return _processor; }
      void setStreamProcessor(StreamProcessor* p);
      virtual void put(boss::Serializable* s)=0;
      virtual ~OutputHandler();
    protected:
      StreamProcessor* _processor;
    private:
      StreamProcessor* _bp;
    };

    class WriterOutputHandler: public OutputHandler {
    public:
      WriterOutputHandler(StreamProcessor* processor_=0, boss::Serializer* ser_=0, int id=-1, boss::IdContext* context = 0);
      inline boss::Serializer* serializer() {return _serializer;}
      inline void setSerializer(boss::Serializer* ser) {_serializer = ser;}
      virtual void put(boss::Serializable* s);
    protected:
      boss::Serializer* _serializer;
    };

    class EnqueuerOutputHandler: public OutputHandler {
    public:
      EnqueuerOutputHandler(StreamProcessor* processor_=0, SerializableList* serializables_=0, int id=-1, boss::IdContext* context = 0);
      inline SerializableList* serializer() {return _serializables;}
      SerializableList * serializables() {return _serializables;}
      void setSerializables(SerializableList * slist) {_serializables = slist;}
      virtual void put(boss::Serializable* s);
    protected:
      SerializableList * _serializables;
    };

    class PropagatorOutputHandler: public OutputHandler {
    public:
      PropagatorOutputHandler(StreamProcessor* processor_=0, StreamProcessor* destinationProcessor_=0, int id=-1, boss::IdContext* context = 0);
      inline StreamProcessor* destinationProcessor() {return _destinationProcessor;}
      inline void setDestinationProcessor(StreamProcessor* dp) {_destinationProcessor = dp;}
      virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
      virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
      virtual void put(boss::Serializable* s);
    protected:
      StreamProcessor* _destinationProcessor;
    };

    friend class OutputHandler;

    StreamProcessor(int id=-1, boss::IdContext* context = 0);
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

    virtual ~StreamProcessor();
    virtual void process(boss::Serializable*s) = 0;
    virtual void put(boss::Serializable* s);
    void setName(const std::string& name_) {_name = name_;}
    const std::string& name() const {return _name;}
  protected:
    std::string _name;
    bool addHandler(OutputHandler* handler_);
    bool removeHandler(OutputHandler* handler_);
    std::list<OutputHandler*>::iterator findHandler(OutputHandler* handler_);
    std::list<OutputHandler*> _handlers;
  };
}
