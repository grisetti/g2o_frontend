#include "persistent_data.h"
#include <assert.h>

BasePersistentData::BasePersistentDataPtr::BasePersistentDataPtr(BasePersistentData* persistentData_) {
  assert (persistentData_);
  _persistentData = persistentData_;
}

BasePersistentData::BasePersistentDataPtr::BasePersistentDataPtr(const BasePersistentData::BasePersistentDataPtr& other) {
  assert (other._persistentData);
  _persistentData = other._persistentData;
  _persistentData->incrementReferences();
}

BasePersistentData::BasePersistentDataPtr& BasePersistentData::BasePersistentDataPtr::operator = (const BasePersistentData::BasePersistentData::BasePersistentDataPtr& other) {
  assert (other->_persistentData);
  assert (_persistentData);
  if (other._persistentData != _persistentData) {
    _persistentData->decrementReferences();
    _persistentData = other._persistentData;
    _persistentData->incrementReferences();
  }
  return *this;
}

BasePersistentData::BasePersistentDataPtr::~BasePersistentDataPtr() {
  _persistentData->decrementReferences();
}
  
  
BasePersistentData::BasePersistentData(const std::string& filename_, size_t offset_){
  _filename = filename_;
  _fileOffset = offset_;
  _references = 0;
  _state = Invalid;
}

BasePersistentData::~BasePersistentData() {
  flush();
  release();
}

void BasePersistentData::release(){
  if (_state & InMemory) {
    releaseImpl();
  }
  _state &= (~InMemory);
}

bool BasePersistentData::retrieve() {
  lock();
  bool ok = state == (~InMemory | Synced);
  if (ok && _state & InMemory ) {
    ok = retrieveImpl();
  }
  if (ok) {
    _state = InMemory | Synced;
  }
  unlock();
  return ok;
}

bool BasePersistentData::flush(){
  bool ok = true;
  lock();
  if (_state & InMemory && ! _references){
    ok = flushImpl();
  } else {
    ok = false;
  }
  if (ok) {
    release();
    _state &= (~InMemory);
  }
  unlock();
  return ok;
}

void BasePersistentData::incrementReferences() {
  assert (_state & InMemory);
  lock();
  _references++;
  unlock();
}

void BasePersistentData::decrementReferences(){
  assert (_state & InMemory && _references);
  lock();
  _references--;
  assert (_references > 0 && "invalid negative number of references");
  if (_references == 0){
    _lastAccessTime = clock();
  }
  unlock();
}
