#include "manifold_voronoi_extractor.h"
namespace manifold_voronoi {

  ManifoldVoronoiData::ManifoldVoronoiData(int id, IdContext* context):
    ImageData(id, context){
    resolution = 0.2;
    node = 0;
  }

  void ManifoldVoronoiData::serialize(ObjectData& data, IdContext& context) {
    ImageData::serialize(data, context);
    data.setFloat("resolution", resolution);
    data.setPointer("node", node);
  }

  void ManifoldVoronoiData::deserialize(ObjectData& data, IdContext& context) {
    ImageData::deserialize(data, context);
    resolution = data.getFloat("resolution");
    data.getReference("node").bind(node);
  }


  ManifoldVoronoiExtractor::ManifoldVoronoiExtractor(int id, boss::IdContext* context):
    StreamProcessor(id,context){
    _manager = 0;
    _cache = 0;
    _resolution = 0.2;
    _xSize = 100;
    _ySize = 100;
    _normalThreshold = 0.64;
  }

  void ManifoldVoronoiExtractor::process(Serializable* s){
    put(s);
    SyncSensorDataNode* n = dynamic_cast<SyncSensorDataNode*>(s);
    int cx = _xSize/2;
    int cy = _ySize/2;
    float ires = 1./_resolution;
    if (n) {
      PwnCloudCache::HandleType h=_cache->get(n);
      CloudWithImageSize* cloud = h.get();
      ManifoldVoronoiData* vdata = new ManifoldVoronoiData();
      vdata->resolution = _resolution;
      boss_map::ImageBLOB* imageBlob = new boss_map::ImageBLOB();
      imageBlob->cvImage().create(100,100, CV_16UC1);
      imageBlob->cvImage().setTo(30000);
      imageBlob->adjustFormat();
      vdata->setTimestamp(0);
      for (size_t i = 0; i<cloud->points().size(); i++){
	pwn::Normal& n = cloud->normals()[i];
	pwn::Point& p = cloud->points()[i];
	int x = cx + p.x()*ires;
	int y = cy + p.y()*ires;
	if ( (x<0) || 
	     (x>=_xSize) || 
	     (y < 0) || 
	     (y>=_ySize) ){
	     continue;
	}
	uint16_t& imZ = imageBlob->cvImage().at<uint16_t>(x,y);
	int pz =  10000 -1000 * p.z();
	if (imZ < pz)
	  continue;
	if (n.squaredNorm()< 0.1)
	  continue;
	if (n.z()<_normalThreshold)
	  continue;
	imZ = pz;
	}
      vdata->imageBlob().set(imageBlob);
      vdata->node = n;
      put(vdata);
    }
  }

  void ManifoldVoronoiExtractor::serialize(ObjectData& data, IdContext& context){
    StreamProcessor::serialize(data,context);
    data.setPointer("manager", _manager);
    data.setPointer("cache", _cache);
    data.setFloat("resolution", _resolution);
    data.setInt("xSize", _xSize);
    data.setInt("ySize", _ySize);
    data.setFloat("normalThreshold", _normalThreshold);
  }

  void ManifoldVoronoiExtractor::deserialize(ObjectData& data, IdContext& context){
    StreamProcessor::deserialize(data,context);
    data.getReference("manager").bind(_manager);
    data.getReference("cache").bind(_cache);
    _resolution = data.getFloat("resolution");
    _xSize = data.getInt("xSize");
    _ySize = data.getInt("ySize");
    _normalThreshold = data.getFloat("normalThreshold");
  }

  BOSS_REGISTER_CLASS(ManifoldVoronoiData);
  BOSS_REGISTER_CLASS(ManifoldVoronoiExtractor);
}
