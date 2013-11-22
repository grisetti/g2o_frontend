#ifndef _PWN_CACHE_H_
#define _PWN_CACHE_H_

#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "pwn_tracker.h"

namespace pwn_tracker {
  using namespace pwn;

  class PwnCache;
  class PwnFrameCacheEntry {
  public:
    friend class PwnCache;
    PwnFrameCacheEntry(PwnCache* cache_, PwnTrackerFrame* frame_);

    pwn::Frame* get(size_t access);

    inline bool isLoaded() const {return _frame->cloud;}

    bool release();

    inline bool isLocked() { return _frame->cloud && _numLocks>0; }

    inline int  numLocks() { return _numLocks; }

    inline void lock() { _numLocks += ( _frame->cloud ? 1 : 0); }

    inline void unlock() { _numLocks --; }

    inline int lastAccess() const {return _lastAccess;}

  protected:
    PwnTrackerFrame* _frame;
    PwnCache* _cache;
    int _numLocks;
    size_t _lastAccess;
  };

  class PwnCache{
  public:
    friend class PwnFrameCacheEntry;
    PwnCache(DepthImageConverter* converter_, int scale_, int maxActiveEntries);
    
    void addEntry(PwnTrackerFrame* frame);

    void removeEntry(PwnTrackerFrame* frame);

    pwn::Frame* get(PwnTrackerFrame* frame);

    inline DepthImageConverter* converter() {return _converter;}
    inline void setConverter(DepthImageConverter* converter_) {_converter = converter_;}

    inline int scale() const {return _scale;}
    inline void setScale(int scale_)  {_scale=scale_;}

    inline int hits() const { return _hits; }
    inline int misses() const { return _misses; }

    bool makeRoom();

    void lock(PwnTrackerFrame* frame);

    void unlock(PwnTrackerFrame* frame);

    
  protected:
    Frame* loadFrame(PwnTrackerFrame* trackerFrame);
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*> _entries;
    std::set<PwnFrameCacheEntry*> _active;
    DepthImageConverter* _converter;
    int _scale;
    size_t _maxElements;
    size_t _lastAccess;
    size_t _hits;
    size_t _misses;
  };


  class PwnCacheHandler: public MapManagerActionHandler {
  public:
    PwnCacheHandler(MapManager* _manager, PwnCache* cache);
    inline PwnCache* cache() {return _cache;}
    virtual ~PwnCacheHandler();
    void init();

    virtual void nodeAdded(MapNode* n);
    virtual void nodeRemoved(MapNode* n);
    virtual void relationAdded(MapNodeRelation* _r);
    virtual void relationRemoved(MapNodeRelation* r);
  protected:
    PwnCache* _cache;
  };

  
}

#endif
