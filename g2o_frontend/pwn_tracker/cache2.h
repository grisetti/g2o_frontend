#pragma once
#include <map>
#include <set>
#include <vector>

namespace pwn_tracker{
  class PwnTrackerFrame;
}

namespace pwn{
  class Frame;
}

namespace cache_ns {
  using namespace std;
  class Cache;
  class CacheEntryHandle;

  
  struct CacheEntry{
  public:
    typedef pwn_tracker::PwnTrackerFrame KeyType;
    typedef pwn::Frame DataType;
    typedef Cache CacheType;
    typedef CacheEntryHandle HandleType;

    friend class CacheEntryHandle;
    friend class Cache;
    CacheEntry(CacheType* c, KeyType* k, DataType* d=0);
    ~CacheEntry();

  
  protected:
    int _numLocks;
    bool _tainted;
    size_t _lastAccess;
    KeyType* _key;
    DataType* _instance;
    CacheType* _cache;


    HandleType get(size_t lastAccess_);
    void release();
    virtual DataType* fetch(KeyType* k);
    virtual bool writeBack(KeyType* k, DataType* d);
  };


  class CacheEntryHandle{
  public:
    typedef CacheEntry EntryType;
    typedef CacheEntry::KeyType KeyType;
    typedef CacheEntry::DataType DataType;
    typedef CacheEntry::CacheType CacheType;
    friend class CacheEntry;

    CacheEntryHandle(const CacheEntryHandle& h) : _entry(h._entry){ _entry-> _numLocks++; }
    ~CacheEntryHandle() {if (_entry) _entry->_numLocks--;}
    CacheEntryHandle& operator = (const CacheEntryHandle&);
    DataType* get();
    void taint();
    inline void release(){_entry = 0;}
    CacheEntryHandle() : _entry(0){}
  protected:
    CacheEntryHandle(EntryType* entry_) : _entry(entry_){ _entry -> _numLocks++; }

    mutable EntryType* _entry;
  };


  class Cache {
  public:
    typedef CacheEntry EntryType;
    typedef EntryType::KeyType KeyType;
    typedef EntryType::DataType DataType;
    typedef EntryType::HandleType HandleType;
    
    Cache(size_t minSlots, size_t maxSlots);
    
    void addEntry(KeyType* k, DataType* d=0);
    void removeEntry(KeyType* k);
    HandleType get(KeyType* k);

    virtual CacheEntry* makeEntry(KeyType* k, DataType* d) = 0;
  protected:
    typedef std::map< KeyType*, EntryType* > KeyEntryMapType;
    typedef std::set< EntryType* > EntrySetType;
    
    EntryType* findEntry(KeyType* k);
    void garbageCollect();
    size_t _maxSlots, _minSlots;
    KeyEntryMapType _entriesMap;
    EntrySetType _activeEntries;
    size_t _lastAccess;

    struct TimeSorter{
      inline bool operator()(const EntryType* e1, const EntryType* e2){
	size_t t1 = (e1->_numLocks) ? 0 : e1->_lastAccess ;
	size_t t2 = (e2->_numLocks) ? 0 : e2->_lastAccess ;
	return t1 > t2;
      }
    };
  };


}
