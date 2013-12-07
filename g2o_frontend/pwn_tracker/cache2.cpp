#include "pwn_tracker.h"
#include "cache2.h"
#include <stdexcept>
#include <algorithm>
namespace cache_ns {

  CacheEntry::CacheEntry(CacheEntry::CacheType* c, CacheEntry::KeyType* k, CacheEntry::DataType* d){
    _numLocks = 0;
    _tainted = 0;
    _lastAccess = 0;
    _key = k;
    _instance = d;
    _cache = c;
  }

  CacheEntry::~CacheEntry() {
    if (_numLocks) {
      throw std::runtime_error("attempt to destroy a cache entry in use");
    }
    if (_instance)
      delete _instance;
  }

  CacheEntry::DataType* CacheEntry::fetch(CacheEntry::KeyType*) { 
    return 0; 
  }

  bool CacheEntry::writeBack(CacheEntry::KeyType*, CacheEntry::DataType*) {
    return false;
  }

  CacheEntry::HandleType CacheEntry::get(size_t lastAccess_){
    _lastAccess = lastAccess_;
    if (!_instance){
      _instance = fetch(_key);
      if (!_instance)
	throw std::runtime_error("error, you should implement the fetch method in the entry type");
    }
    return CacheEntry::HandleType(this);
  };

  void CacheEntry::release(){
    cerr << " releasing " << _key << endl;
    if (_numLocks>0){
      throw std::runtime_error("attempt to release entry in use");
    }
    if (_tainted && _instance){
      if (! writeBack(_key, _instance) ){
	throw std::runtime_error("error, you should implement the writeBack method in the entry type");
      }
    }
    delete _instance;
    _instance = 0;
    _tainted = 0;
  }


  CacheEntryHandle& CacheEntryHandle::operator = (const CacheEntryHandle& c2){
    if (c2._entry == _entry)
      return *this;

    if (_entry && _entry->_numLocks>0){
      _entry->_numLocks--;
    }
    _entry = c2._entry;
    if (_entry)
      _entry->_numLocks++;
    return *this;
  }


  CacheEntryHandle::DataType* CacheEntryHandle::get() { 
    if (!_entry)
      throw std::runtime_error("cache handle, invalid get on null entry");
    return _entry->_instance; 
  }

  void CacheEntryHandle::taint() { 
    if (!_entry)
      throw std::runtime_error("cannot taint a null entry");
    if (!_entry->_instance)
      throw std::runtime_error("cannot taint a non fetched thing");
    _entry->_tainted = true; 
  }

  Cache::Cache(size_t minSlots, size_t maxSlots) {
    _maxSlots = maxSlots;
    _minSlots = minSlots;
    _lastAccess=0;
  }

  void Cache::addEntry(Cache::KeyType* k, Cache::DataType* d){
    cerr << "addind entry for frame: " << k << endl;
    Cache::EntryType* e = makeEntry(k,d);
    _entriesMap.insert(make_pair(k, e));
    if (e->_instance){
      e->get(_lastAccess++);
      cerr << "getting thing in pool: " << k << endl;
    }
  }

  void Cache::removeEntry(Cache::KeyType* k){
    EntryType* e = findEntry(k);
    if (e){
      _activeEntries.erase(e);
    }
    _entriesMap.erase(k);
    delete e;
  }

  Cache::HandleType Cache::get(Cache::KeyType* k){
    cerr << "seeking entry for frame: " << k << endl;
    Cache::EntryType* e = findEntry(k);
    _activeEntries.insert(e);
    Cache::HandleType h = e->get(_lastAccess++);
    cerr << "e->_instance: " << e->_instance << " e->_locks:" << e->_numLocks << endl; 
    garbageCollect();
    return h;
  }

  
  Cache::EntryType* Cache::findEntry(Cache::KeyType* k){
    std::map<KeyType*, EntryType*>::iterator it=_entriesMap.find(k);
    if (it == _entriesMap.end())
      return 0;
    return it->second;
  }

  void Cache::garbageCollect(){
    if (_activeEntries.size()<_maxSlots)
      return;

    cerr << "garbage collecting, size = " << _activeEntries.size() << endl;
    // copy all elements in the active pool
    std::vector<Cache::EntryType*> v(_activeEntries.size());
    int k=0;
    int locked = 0;
    for (Cache::EntrySetType::iterator it=_activeEntries.begin(); it!=_activeEntries.end(); it++){
      v[k++]=*it;
      if ((*it)->_numLocks)
	locked ++;
    }
    cerr << "locked entries: " << locked << endl; 
    std::sort(v.begin(), v.end(), Cache::TimeSorter());

    int toKill= v.size()-_minSlots;
    for (size_t i=0; i<v.size() && toKill; i++){
      Cache::EntryType* e=v[i];
      e->release();
      _activeEntries.erase(e);
      toKill--;
    }
    if (_activeEntries.size()>_maxSlots) {
      throw std::runtime_error("Cache is too small");
    }
  }

}
