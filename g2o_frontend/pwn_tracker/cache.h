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
    PwnFrameCacheEntry(PwnCache* cache_, PwnTrackerFrame* frame_);

    pwn::Frame* get(size_t access);

    inline bool isLoaded() const {return _instance;}

    bool release();

    inline bool isLocked() { return _instance && _isLocked; }

    inline void lock() { _isLocked = _instance; }

    inline void unlock() { _isLocked = false; }

    inline int lastAccess() const {return _lastAccess;}

  protected:
    pwn::Frame* _instance;
    PwnTrackerFrame* _frame;
    PwnCache* _cache;
    bool _isLocked;
    size_t _lastAccess;
  };

  class PwnCache{
  public:
    friend class PwnFrameCacheEntry;
    PwnCache(DepthImageConverter* converter_, int scale_, int maxActiveEntries);
    
    bool addEntry(PwnTrackerFrame* frame);

    pwn::Frame* get(PwnTrackerFrame* frame);

    inline DepthImageConverter* converter() {return _converter;}
    inline int scale() const {return _scale;}
    inline int hits() const { return _hits; }
    inline int misses() const { return _misses; }

    bool makeRoom();

    void lock(PwnTrackerFrame* frame);

    void unlock(PwnTrackerFrame* frame);

    
  protected:
    Frame* makeFrame(PwnTrackerFrame* trackerFrame);
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*> _entries;
    std::set<PwnFrameCacheEntry*> _active;
    DepthImageConverter* _converter;
    int _scale;
    size_t _maxElements;
    size_t _lastAccess;
    size_t _hits;
    size_t _misses;
  };

}

#endif
