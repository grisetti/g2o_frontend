#include "pwn_tracker_cache.h"
#include "pwn_tracker.h"

namespace pwn_tracker{
  using namespace cache_ns;
  PwnCacheEntry::PwnCacheEntry(PwnCache* c, PwnTrackerFrame* k, pwn::Frame* d):
    CacheEntry<PwnTrackerFrame,pwn::Frame>(k,d){
    _pwnCache=c;
  }

  pwn::Frame* PwnCacheEntry::fetch(CacheEntry::KeyType* k){
    return _pwnCache->loadFrame(k);
  }

  PwnCache::PwnCache(DepthImageConverter* converter_, int scale_, int minSlots_, int maxSlots_):
    Cache<PwnCacheEntry>(minSlots_, maxSlots_), _converter(converter_), _scale(scale_){
  }


  Frame* PwnCache::loadFrame(PwnTrackerFrame* trackerFrame){
    boss_map::ImageBLOB* depthBLOB = trackerFrame->depthImage.get();
    PinholePointProjector* projector = (PinholePointProjector*)_converter->projector();
    projector->setImageSize(trackerFrame->imageRows, trackerFrame->imageCols);
    pwn::DepthImage depth;
    pwn::Frame* cloud=new pwn::Frame;
    Eigen::Matrix3f cameraMatrix;
    Eigen::Isometry3f offset;
    depth.fromCvMat(depthBLOB->cvImage());
    convertScalar(offset, trackerFrame->sensorOffset);
    convertScalar(cameraMatrix, trackerFrame->cameraMatrix);
    cameraMatrix(2,2)=1;
    projector->setCameraMatrix(cameraMatrix);
    pwn::DepthImage scaledDepth;
    DepthImage::scale(scaledDepth, depth, _scale);
    projector->scale (1./_scale);
    _converter->compute(*cloud, scaledDepth, offset);
    trackerFrame->depthImage.set(0);
    //delete depthBLOB;
    return cloud;
  }

  Cache<PwnCacheEntry>::EntryType* PwnCache::makeEntry(KeyType* k, DataType*) {
    return new PwnCacheEntry(this, k,k->cloud);
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

