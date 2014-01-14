#pragma once
#include <map>
#include <set>
#include <vector>


namespace cache_ns {
  using namespace std;
  template <typename EntryType_>
  class Cache;

  template <typename EntryType_>
  class CacheEntryHandle;

  
  template <typename KeyType_, typename DataType_>
  class CacheEntry{
  public:
    typedef KeyType_ KeyType;
    typedef  DataType_ DataType;
    typedef CacheEntryHandle< CacheEntry<KeyType_, DataType_> > HandleType;

    friend class Cache< CacheEntry<KeyType_, DataType_> >;
    friend class CacheEntryHandle< CacheEntry<KeyType_, DataType_> >;
    CacheEntry(KeyType* k, DataType* d=0);
    ~CacheEntry();

  
    //protected:
    int _numLocks;
    bool _tainted;
    size_t _lastAccess;
    KeyType* _key;
    DataType* _instance;


    HandleType get(size_t lastAccess_);
    void release();
    virtual DataType* fetch( KeyType* k);
    virtual bool writeBack(  KeyType* k, DataType* d);
  };


  template <typename EntryType_>
  class CacheEntryHandle{
  public:
    typedef EntryType_ EntryType;
    typedef typename EntryType_::KeyType KeyType;
    typedef typename EntryType_::DataType DataType;

    CacheEntryHandle(const CacheEntryHandle& h) : _entry(h._entry){ _entry-> _numLocks++; }
    ~CacheEntryHandle() {if (_entry) _entry->_numLocks--;}
    CacheEntryHandle& operator = (const CacheEntryHandle&);
    DataType* get();
    void taint();
    inline void release(){_entry = 0;}
    CacheEntryHandle() : _entry(0){}
    CacheEntryHandle(EntryType* entry_) : _entry(entry_){ _entry -> _numLocks++; }
  protected:

    mutable EntryType* _entry;
  };


  template <typename EntryType_>
  class Cache {
  public:
    typedef EntryType_ EntryType;
    typedef typename EntryType_::KeyType KeyType;
    typedef typename EntryType_::DataType DataType;
    typedef typename EntryType_::HandleType HandleType;
    
    Cache(size_t minSlots, size_t maxSlots);
    
    void addEntry(KeyType* k, DataType* d=0);
    void removeEntry(KeyType* k);
    HandleType get(KeyType* k);

    virtual EntryType* makeEntry( KeyType* k, DataType* d) = 0;
    virtual ~Cache() {}
    int hits() const {return _hits;}
    int misses() const {return _misses;}

    size_t minSlots() const {return _minSlots;}
    size_t maxSlots() const {return _maxSlots;}
    void setMinSlots(size_t minSlots_) {_minSlots = minSlots_;}
    void setMaxSlots(size_t maxSlots_) {_maxSlots = maxSlots_;}
    
  protected:
    typedef std::map< KeyType*,  EntryType* > KeyEntryMapType;
    typedef std::set< EntryType* > EntrySetType;
    size_t _maxSlots, _minSlots;
    
    EntryType* findEntry( KeyType* k);
    void garbageCollect();
    KeyEntryMapType _entriesMap;
    EntrySetType _activeEntries;
    size_t _lastAccess;
    int _hits;
    int _misses;
    struct TimeSorter{
      inline bool operator()(const EntryType* e1, const EntryType* e2){
	size_t t1 = (e1->_numLocks) ? std::numeric_limits<size_t>::max() : e1->_lastAccess ;
	size_t t2 = (e2->_numLocks) ? std::numeric_limits<size_t>::max() : e2->_lastAccess ;
	return t1 < t2;
      }
    };
  };


}

#include "cache.hpp"
