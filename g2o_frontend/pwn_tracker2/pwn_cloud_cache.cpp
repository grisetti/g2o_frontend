#include "pwn_cloud_cache.h"
#include "g2o_frontend/pwn_core/pwn_static.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include "pwn_matcher_base.h"

namespace pwn_tracker{
  using namespace cache_ns;
  PwnCloudCacheEntry::PwnCloudCacheEntry(PwnCloudCache* c, SyncSensorDataNode* k, pwn::Cloud* d):
    CacheEntry<SyncSensorDataNode,pwn::Cloud>(k,d){
    _pwnCache=c;
  }

  pwn::Cloud* PwnCloudCacheEntry::fetch(CacheEntry::KeyType* k){
    return _pwnCache->loadCloud(k);
  }

  PwnCloudCache::PwnCloudCache(DepthImageConverter* converter_, 
			       RobotConfiguration* robotConfiguration_,
			       const std::string& topic_,
			       int scale_, int minSlots_, int maxSlots_):
    Cache<PwnCloudCacheEntry>(minSlots_, maxSlots_), _converter(converter_), _scale(scale_){
    numCalls = 0;
    cumTime = 0;
    _topic = topic_;
    _robotConfiguration = robotConfiguration_;
  }


  pwn::Cloud* PwnCloudCache::loadCloud(SyncSensorDataNode* trackerNode){
    PinholeImageData* imdata = trackerNode->sensorData()->sensorData<PinholeImageData>(_topic);
    if (! imdata) {
      throw std::runtime_error("the required topic does not match the requested type");
    }

    boss_map::ImageBLOB* depthBLOB = imdata->imageBlob().get();

    
    Eigen::Isometry3d sensorOffset = _robotConfiguration->sensorOffset(imdata->sensor());

    PinholePointProjector* projector = dynamic_cast<PinholePointProjector*>(_converter->projector());
    projector->setImageSize(depthBLOB->cvImage().rows, depthBLOB->cvImage().cols);
    pwn::DepthImage depth;
    pwn::Cloud* cloud=new pwn::Cloud;
    Eigen::Matrix3f cameraMatrix;
    Eigen::Isometry3f offset;
    DepthImage_convert_16UC1_to_32FC1(depth, depthBLOB->cvImage()); 
    //depth.fromCvMat(depthBLOB->cvImage());
    convertScalar(offset, sensorOffset);
    convertScalar(cameraMatrix, imdata->cameraMatrix());
    cameraMatrix(2,2)=1;
    projector->setCameraMatrix(cameraMatrix);
    pwn::DepthImage scaledDepth;
    DepthImage_scale(scaledDepth, depth, _scale);
    projector->scale (1./_scale);
    double t0 = g2o::get_time();
    _converter->compute(*cloud, scaledDepth, offset);
    double t1 = g2o::get_time();

    imdata->imageBlob().set(0);
    //delete depthBLOB;
    numCalls ++;
    cumTime += (t1-t0);
    return cloud;
  }

  Cache<PwnCloudCacheEntry>::EntryType* PwnCloudCache::makeEntry(KeyType* k, DataType*) {
    return new PwnCloudCacheEntry(this, k);
  }

  PwnCloudCacheHandler::PwnCloudCacheHandler(MapManager* manager_, PwnCloudCache* cache_):
    MapManagerActionHandler(manager_){
    _cache = cache_;
  }

  PwnCloudCacheHandler::~PwnCloudCacheHandler() {}
  void PwnCloudCacheHandler::init(){
    for (std::set<MapNode*>::iterator it = _manager->nodes().begin(); it!=_manager->nodes().end(); it++){
      nodeAdded(*it);
    }
  }

  void PwnCloudCacheHandler::nodeAdded(MapNode* n) {
    SyncSensorDataNode* f = dynamic_cast<SyncSensorDataNode*>(n);
    if (f)
      _cache->addEntry(f);
  }

  void PwnCloudCacheHandler::nodeRemoved(MapNode* n) {
    SyncSensorDataNode* f = dynamic_cast<SyncSensorDataNode*>(n);
    if (f)
      _cache->removeEntry(f);
  }

  void PwnCloudCacheHandler::relationAdded(MapNodeRelation* ) {}
  void PwnCloudCacheHandler::relationRemoved(MapNodeRelation* ) {}

} // end namespace

