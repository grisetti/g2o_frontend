#include "cache.h"

namespace pwn_tracker{

  PwnFrameCacheEntry::PwnFrameCacheEntry(PwnCache* cache_, PwnTrackerFrame* frame_){
    _frame = frame_;
    _cache = cache_;
    _instance = 0;
    _isLocked = 0;
    _lastAccess = 0;
  }


  bool PwnFrameCacheEntry::release() {
    if (_instance && ! _isLocked) {
      delete _instance; 
      _instance = 0; 
      return true;
    } 
    return false;
  }

  pwn::Frame* PwnFrameCacheEntry::get(size_t access) {
    if (_instance)
      return _instance;
    _instance = _cache->makeFrame(_frame);
    //cerr << "loading frame: " << _instance  << endl;
    _lastAccess = access;
    return _instance;
  }

  PwnCache::PwnCache(DepthImageConverter* converter_, int scale_, int maxActiveEntries){
    _converter = converter_;
    _scale = scale_;
    _lastAccess = 0;
    _hits = 0;
    _misses = 0;
    _maxElements = maxActiveEntries;
  }

  bool PwnCache::addEntry(PwnTrackerFrame* frame){
    //cerr << "adding entry for frame" << frame << endl;
    if (_entries.count(frame))
      return false;
    PwnFrameCacheEntry* entry = new PwnFrameCacheEntry(this, frame);
    _entries.insert(make_pair(frame, entry));
    //cerr << "DONE" << frame << endl;
    return true;
  }

  pwn::Frame* PwnCache::get(PwnTrackerFrame* frame) {
    // seek if you have it in the entries;
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*>::iterator it = _entries.find(frame);
    if (it==_entries.end())
      return 0;
    PwnFrameCacheEntry* entry = it->second;
    if (entry->isLoaded()){
      _hits++;
      return entry->get(_lastAccess++);
    } 
    if (_active.size()>_maxElements){
      if (! makeRoom() )
	throw std::runtime_error("no room in cache, no disposable element. Aborting");
    }
    //cerr << "inserting new entry in the pool" << endl;
    _misses++;
    _active.insert(entry);
    return entry->get(_lastAccess++);
  }


  bool  PwnCache::makeRoom() {
    // seek for the oldest element in the set that is not locked
    PwnFrameCacheEntry* oldest = 0;
    for (std::set<PwnFrameCacheEntry*>::iterator it=_active.begin(); it!=_active.end(); it++){
      PwnFrameCacheEntry* entry = *it;
      if (entry->isLocked())
	continue;
      if(! oldest) {
	oldest = entry;
	continue;
      }
      if (oldest->lastAccess()>entry->lastAccess())
	oldest = entry;
    }
    if (oldest) {
      oldest->release();
      _active.erase(oldest);
      return true;
    }
    return false;
  }

  void  PwnCache::lock(PwnTrackerFrame* frame) {
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*>::iterator it = _entries.find(frame);
    if (it==_entries.end())
      return;
    it->second->lock();
  }

  void  PwnCache::unlock(PwnTrackerFrame* frame) {
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*>::iterator it = _entries.find(frame);
    if (it==_entries.end())
      return;
    it->second->unlock();
  }

  Frame* PwnCache::makeFrame(PwnTrackerFrame* trackerFrame){
    boss_logger::ImageBLOB* depthBLOB = trackerFrame->depthImage.get();
    PinholePointProjector* projector = (PinholePointProjector*)_converter->_projector;
    projector->setImageSize(trackerFrame->imageRows, trackerFrame->imageCols);
    pwn::DepthImage depth;
    pwn::Frame* cloud=new pwn::Frame;
    Eigen::Matrix3f cameraMatrix;
    Eigen::Isometry3f offset;
    depth.fromCvMat(depthBLOB->cvImage());
    convertScalar(offset, trackerFrame->sensorOffset);
    convertScalar(cameraMatrix, trackerFrame->cameraMatrix);
    projector->setCameraMatrix(cameraMatrix);
    pwn::DepthImage scaledDepth;
    DepthImage::scale(scaledDepth, depth, _scale);
    projector->scale (1./_scale);
    _converter->compute(*cloud, scaledDepth, offset);
    delete depthBLOB;
    return cloud;
  }

} // end namespace

