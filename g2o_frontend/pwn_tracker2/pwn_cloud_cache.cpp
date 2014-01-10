#include "pwn_cloud_cache.h"
#include "g2o_frontend/pwn_core/pwn_static.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include "pwn_matcher_base.h"

namespace pwn_tracker{
  using namespace cache_ns;
  PwnCloudCacheEntry::PwnCloudCacheEntry(PwnCloudCache* c, SyncSensorDataNode* k, CloudWithImageSize* d):
    CacheEntry<SyncSensorDataNode,CloudWithImageSize>(k,d){
    _pwnCache=c;
  }

  CloudWithImageSize* PwnCloudCacheEntry::fetch(CacheEntry::KeyType* k){
    return _pwnCache->loadCloud(k);
  }

  PwnCloudCache::PwnCloudCache(DepthImageConverter* converter_, 
			       RobotConfiguration* robotConfiguration_,
			       const std::string& topic_,
			       int scale_, int minSlots_, int maxSlots_,
			       int id, boss::IdContext* context):
    Cache<PwnCloudCacheEntry>(minSlots_, maxSlots_), 
    Identifiable(id, context), _converter(converter_), _scale(scale_){
    cerr << "cache construct" << endl;
    numCalls = 0;
    cumTime = 0;
    _topic = topic_;
    _robotConfiguration = robotConfiguration_;
  }

  void PwnCloudCache::serialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::serialize(data,context);
    _tempConverter = 0;
    if(_converter)
      _tempConverter = dynamic_cast<pwn_boss::DepthImageConverter*>(_converter);
    data.setPointer("converter", _tempConverter);
    data.setInt("scale", _scale);
    data.setString("topic", _topic);
    data.setInt("minSlots", _minSlots);
    data.setInt("maxSlots", _maxSlots);
  }
  
  void PwnCloudCache::deserialize(boss::ObjectData& data, boss::IdContext& context){
    cerr << "deserializing cache" << endl;
    Identifiable::deserialize(data,context);
    _tempConverter = 0;
    _converter = 0;
    data.getReference("converter").bind(_tempConverter);
    _scale = data.getInt("scale");
    _topic = data.getString("topic");
    _minSlots = data.getInt("minSlots");
    _maxSlots = data.getInt("maxSlots");
  }

  void PwnCloudCache::deserializeComplete() {
    _converter = _tempConverter;
  }
  
  CloudWithImageSize* PwnCloudCache::loadCloud(SyncSensorDataNode* trackerNode){
    PinholeImageData* imdata = trackerNode->sensorData()->sensorData<PinholeImageData>(_topic);
    if (! imdata) {
      for(size_t i =0; i<trackerNode->sensorData()->sensorDatas.size(); i++){
	cerr << trackerNode->sensorData()->sensorDatas[i]->className() << " " <<  trackerNode->sensorData()->sensorDatas[i]->topic() << endl;
      }
      std::string err("the required topic does not match the requested type (should be PinholeImageData, it is");
      err += trackerNode->sensorData()->className();
      throw std::runtime_error(err.c_str());
    }

    boss_map::ImageBLOB* depthBLOB = imdata->imageBlob().get();

    
    Eigen::Isometry3d sensorOffset = _robotConfiguration->sensorOffset(imdata->sensor());

    PinholePointProjector* projector = dynamic_cast<PinholePointProjector*>(_converter->projector());
    projector->setImageSize(depthBLOB->cvImage().rows, depthBLOB->cvImage().cols);
    pwn::DepthImage depth;
    CloudWithImageSize* cloud=new CloudWithImageSize;
    cloud->imageRows = depthBLOB->cvImage().rows;
    cloud->imageCols = depthBLOB->cvImage().cols;
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

  PwnCloudCacheHandler::PwnCloudCacheHandler(MapManager* manager_, PwnCloudCache* cache_,
					     int id, boss::IdContext* context):
    MapManagerActionHandler(manager_, id, context){
    _cache = cache_;
  }

  PwnCloudCacheHandler::~PwnCloudCacheHandler() {}
  void PwnCloudCacheHandler::init(){
    cerr << "Manager: " << _manager << endl;
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

  void PwnCloudCacheHandler::serialize(boss::ObjectData& data, boss::IdContext& context){
    MapManagerActionHandler::serialize(data,context);
    data.setPointer("cache", _cache);
  }
  
  void PwnCloudCacheHandler::deserialize(boss::ObjectData& data, boss::IdContext& context){
    MapManagerActionHandler::deserialize(data,context);
    data.getReference("cache").bind(_cache);
  }

  void PwnCloudCacheHandler::deserializeComplete() {
    init();
  }


  BOSS_REGISTER_CLASS(PwnCloudCache);
  BOSS_REGISTER_CLASS(PwnCloudCacheHandler);
} // end namespace

