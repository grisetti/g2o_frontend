#include <stdexcept>
#include <algorithm>
#include <iostream>

namespace cache_ns {
  template <typename KeyType_, typename DataType_>
  CacheEntry<KeyType_, DataType_>::CacheEntry(KeyType* k, 
					      DataType* d){
    _numLocks = 0;
    _tainted = 0;
    _lastAccess = 0;
    _key = k;
    _instance = d;
  }

  template <typename KeyType_, typename DataType_>
  CacheEntry<KeyType_, DataType_>::~CacheEntry() {
    if (_numLocks) {
      throw std::runtime_error("attempt to destroy a cache entry in use");
    }
    if (_instance)
      delete _instance;
  }

  template <typename KeyType_, typename DataType_>
  typename CacheEntry<KeyType_, DataType_>::DataType* CacheEntry<KeyType_, DataType_>::fetch(typename CacheEntry<KeyType_, DataType_>::KeyType*) { 
    return 0; 
  }

  template <typename KeyType_, typename DataType_>
  bool CacheEntry<KeyType_, DataType_>::writeBack(typename CacheEntry<KeyType_, DataType_>::KeyType*, 
						  typename CacheEntry<KeyType_, DataType_>::DataType*) {
    return false;
  }

  template <typename KeyType_, typename DataType_>
  typename CacheEntry<KeyType_, DataType_>::HandleType CacheEntry<KeyType_, DataType_>::get(size_t lastAccess_){
    _lastAccess = lastAccess_;
    if (!_instance){
      _instance = fetch(_key);
      if (!_instance)
	throw std::runtime_error("error, you should implement the fetch method in the entry type");
    }
    return typename CacheEntry<KeyType_, DataType_>::HandleType(this);
  };

  template <typename KeyType_, typename DataType_>
  void CacheEntry<KeyType_, DataType_>::release(){
    // cerr << " releasing " << _key << endl;
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


  template  <typename EntryType_>
  CacheEntryHandle<EntryType_>& CacheEntryHandle<EntryType_>::operator = (const CacheEntryHandle<EntryType_>& c2){
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


  template <typename EntryType_>
  typename CacheEntryHandle<EntryType_>::DataType* CacheEntryHandle<EntryType_>::get() { 
    if (!_entry)
      throw std::runtime_error("cache handle, invalid get on null entry");
    return _entry->_instance; 
  }

  template  <typename EntryType_>
  void CacheEntryHandle<EntryType_>::taint() { 
    if (!_entry)
      throw std::runtime_error("cannot taint a null entry");
    if (!_entry->_instance)
      throw std::runtime_error("cannot taint a non fetched thing");
    _entry->_tainted = true; 
  }

  template <typename  EntryType_>
  Cache<EntryType_>::Cache(size_t minSlots, size_t maxSlots) {
    _maxSlots = maxSlots;
    _minSlots = minSlots;
    _lastAccess=0;
  }

  template <typename EntryType_>
  void Cache<EntryType_>::addEntry(typename Cache<EntryType_>::KeyType* k, typename Cache<EntryType_>::DataType* d){
    // cerr << "addind entry for frame: " << k << endl;
    typename Cache<EntryType_>::EntryType* e = makeEntry(k,d);
    _entriesMap.insert(make_pair(k, e));
    if (e->_instance){
      e->get(_lastAccess++);
      // cerr << "getting thing in pool: " << k << endl;
    }
  }

  template  <typename EntryType_>
  void Cache<EntryType_>::removeEntry(Cache::KeyType* k){
    typename Cache<EntryType_>::EntryType* e = findEntry(k);
    if (e){
      _activeEntries.erase(e);
    }
    _entriesMap.erase(k);
    delete e;
  }

  template <typename EntryType_>
  typename Cache<EntryType_>::HandleType Cache<EntryType_>::get(typename Cache<EntryType_>::KeyType* k){
    // cerr << "seeking entry for frame: " << k << endl;
    typename Cache<EntryType_>::EntryType* e = findEntry(k);
    _activeEntries.insert(e);
    typename Cache<EntryType_>::HandleType h = e->get(_lastAccess++);
    // cerr << "e->_instance: " << e->_instance << " e->_locks:" << e->_numLocks << endl; 
    garbageCollect();
    return h;
  }

  
  template <typename EntryType_>
  typename Cache<EntryType_>::EntryType* Cache<EntryType_>::findEntry(Cache<EntryType_>::KeyType* k){
    typename Cache<EntryType_>::KeyEntryMapType::iterator it=_entriesMap.find(k);
    if (it == _entriesMap.end())
      return 0;
    return it->second;
  }

  template <typename EntryType_>
  void Cache<EntryType_>::garbageCollect(){
    if (_activeEntries.size()<_maxSlots)
      return;

    // cerr << "garbage collecting, size = " << _activeEntries.size() << endl;
    // copy all elements in the active pool
    std::vector<Cache::EntryType*> v(_activeEntries.size());
    int k=0;
    int locked = 0;
    for (typename Cache<EntryType_>::EntrySetType::iterator it=_activeEntries.begin(); it!=_activeEntries.end(); it++){
      v[k++]=*it;
      if ((*it)->_numLocks)
	locked ++;
    }
    // cerr << "locked entries: " << locked << endl; 
    std::sort(v.begin(), v.end(), Cache<EntryType_>::TimeSorter());

    int toKill= v.size()-_minSlots;
    for (size_t i=0; i<v.size() && toKill; i++){
      typename Cache<EntryType_>::EntryType* e=v[i];
      e->release();
      _activeEntries.erase(e);
      toKill--;
    }
    if (_activeEntries.size()>_maxSlots) {
      throw std::runtime_error("Cache is too small");
    }
  }

}
