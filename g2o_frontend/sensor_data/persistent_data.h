#ifndef _PERSISTENT_DATA_H_
#define _PERSISTENT_DATA_H_

#include <set>
#include <string>
#include<ctime>
#include "mutexed.h"

/**
a persistent data is something that lives on the disk, and can be loaded and modifiedin memory.
During its existance, it is associated to a file, and the memory and disk states should be consistent.

*/
class BasePersistentData : public Mutexed {
public:
  struct BasePersistentDataPtr{
  protected:
    BasePersistentDataPtr(BasePersistentData* persistentData_);

  public:
    BasePersistentDataPtr(const BasePersistentDataPtr& other);

    BasePersistentDataPtr& operator = (const BasePersistentDataPtr& other);

    virtual ~BasePersistentDataPtr();
  protected:
    clock_t _creationTime;
    mutable BasePersistentData* _persistentData;
  };
  
  enum State {
    Invalid  = 0x0,
    InMemory = 0x1,  // this flag is set if it is loaded in main memory
    Synced  =  0x2}; // this flag is set if the data on disk is consistent with the memory copy
  
  BasePersistentData(const std::string& filename_, size_t offset=size_t(0));
  virtual ~BasePersistentData();
  bool retrieve();
  bool flush();
  bool release();
  inline const std::string& filename() const {return _filename;}
  inline size_t fileOffset() const {return _fileOffset;}
  inline size_t references() const { return _references;}
  inline int state() const { return _state;}
  void incrementReferences();
  void decrementReferences();
protected:
  virtual bool retrieveImpl()=0;
  virtual bool flushImpl()=0;
  virtual bool releaseImpl() = 0;
  int _state;
  std::string _filename;
  size_t _fileOffset;
  clock_t _lastAccessTime;
  int _references;
};

template <typename BaseData>
class PersistentData: public BasePersistentData{
public:
  struct PersistentDataPtr: public BasePersistentData::BasePersistentDataPtr {
  protected:
    PersistentDataPtr(PersistentData* persistentData_): BasePersistentDataPtr(persistentData_){}
  public:
    PersistentDataPtr(const PersistentDataPtr& other): BasePersistentDataPtr(other){}
    PersistentDataPtr& operator = (const BasePersistentDataPtr& other){
      BasePersistentDataPtr::operator=(other);
      return *this;
    }

    BaseData& operator *() { 
      _persistentData->_state &= (~Synced);
      return *static_cast<PersistentData*>(_persistentData)->data();
    }
    
    const BaseData& operator *() const {
      return *static_cast<const PersistentData*>(_persistentData)->data();
    }
  };

  PersistentData(const std::string& filename_, size_t& offset){
    _data = 0;
  }
  
  void setData(BaseData* d){
    if (d == _data)
      return;
    releaseImpl();
    _data = d;
    _state = InMemory & ~Synced;
  }

  void releaseImpl() {
    assert (!_references && _state & InMemory && "cannot delete this data, pending instances to ptr are still alive");
    if (_data)
      delete _data;
    _data = 0;
  }

  PersistentDataPtr ptr(){
    if (!_state) {
      assert (0 && "cannot return an handle to an invalid object");
    }
    if (!(_state & InMemory) && retrieve()){
      _state |= (InMemory|Synced);
    }
    return PersistentDataPtr(this);
  }
  

protected:
  BaseData* _data;
};

#endif
