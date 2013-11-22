#include "cache.h"

namespace pwn_tracker{

  PwnFrameCacheEntry::PwnFrameCacheEntry(PwnCache* cache_, PwnTrackerFrame* frame_){
    _frame = frame_;
    _cache = cache_;
    _numLocks = 0;
    _lastAccess = 0;
  }


  bool PwnFrameCacheEntry::release() {
    if (_frame->cloud && _numLocks>=0) {
      delete _frame->cloud; 
      _frame->cloud = 0;  
      return true;
    } 
    return false;
  }

  pwn::Frame* PwnFrameCacheEntry::get(size_t access) {
    if (_frame->cloud)
      return _frame->cloud;
    _frame->cloud = _cache->loadFrame(_frame);
    //cerr << "loading frame: " << _frame->cloud  << endl;
    _lastAccess = access;
    return _frame->cloud;
  }

  PwnCache::PwnCache(DepthImageConverter* converter_, int scale_, int maxActiveEntries){
    _converter = converter_;
    _scale = scale_;
    _lastAccess = 0;
    _hits = 0;
    _misses = 0;
    _maxElements = maxActiveEntries;
  }

  void PwnCache::addEntry(PwnTrackerFrame* frame){
    if (_entries.count(frame))
	throw std::runtime_error("entry already in cache, aborting.");
    PwnFrameCacheEntry* entry = new PwnFrameCacheEntry(this, frame);
    _entries.insert(make_pair(frame, entry));
    if (frame->cloud){
      if (_active.size()>_maxElements){
	if (! makeRoom() )
	  throw std::runtime_error("no room in cache, no disposable element. Aborting");
      }
      entry->_lastAccess = _lastAccess++;
      _active.insert(entry);
    }
  }
  

  void PwnCache::removeEntry(PwnTrackerFrame* frame){
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*>::iterator it = _entries.find(frame);
    if (it==_entries.end()) {
	throw std::runtime_error("no entry in cache. Aborting");
    }
    if(it->second->isLocked())
	throw std::runtime_error("no entry in cache. illegal delete");
    _entries.erase(it);
    PwnFrameCacheEntry* e = it->second;
    _active.erase(it->second);
    delete e;
  }


  pwn::Frame* PwnCache::get(PwnTrackerFrame* frame) {
    // seek if you have it in the entries;
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*>::iterator it = _entries.find(frame);
    if (it==_entries.end()) {
	throw std::runtime_error("no entry in cache. Aborting");
	return 0;
    }
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

  Frame* PwnCache::loadFrame(PwnTrackerFrame* trackerFrame){
    boss_logger::ImageBLOB* depthBLOB = trackerFrame->depthImage.get();
    PinholePointProjector* projector = (PinholePointProjector*)_converter->projector();
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
    trackerFrame->depthImage.set(0);
    //delete depthBLOB;
    return cloud;
  }

  PwnCacheHandler::PwnCacheHandler(MapManager* manager_, PwnCache* cache_):
    MapManagerActionHandler(manager_){
    _cache = cache_;
  }

  PwnCacheHandler::~PwnCacheHandler() {}
  void PwnCacheHandler::init(){
    for (std::set<MapNode*>::iterator it = _manager->nodes().begin(); it!=_manager->nodes().end(); it++){
      nodeAdded(*it);
    }
  }

  void PwnCacheHandler::nodeAdded(MapNode* n) {
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(n);
    if (f)
      _cache->addEntry(f);
  }

  void PwnCacheHandler::nodeRemoved(MapNode* n) {
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(n);
    if (f)
      _cache->removeEntry(f);
  }

  void PwnCacheHandler::relationAdded(MapNodeRelation* ) {}
  void PwnCacheHandler::relationRemoved(MapNodeRelation* ) {}

} // end namespace

