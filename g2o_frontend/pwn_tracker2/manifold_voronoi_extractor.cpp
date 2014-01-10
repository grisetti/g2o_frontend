#include "manifold_voronoi_extractor.h"
namespace manifold_voronoi {

  ManifoldVoronoiExtractor::ManifoldVoronoiExtractor(int id, boss::IdContext* context):
    StreamProcessor(id,context){
    _manager = 0;
    _cache = 0;
  }

  void ManifoldVoronoiExtractor::process(Serializable* s){
    put(s);
    SyncSensorDataNode* n = dynamic_cast<SyncSensorDataNode*>(s);
    if (n) {
      PwnCloudCache::HandleType h=_cache->get(n);
      CloudWithImageSize* cloud = h.get();
      cerr << "Taigo, I have a cloud" << endl;
    }
  }

  void ManifoldVoronoiExtractor::serialize(ObjectData& data, IdContext& context){
    StreamProcessor::serialize(data,context);
    data.setPointer("manager", _manager);
    data.setPointer("cache", _cache);
  }

  void ManifoldVoronoiExtractor::deserialize(ObjectData& data, IdContext& context){
    StreamProcessor::deserialize(data,context);
    data.getReference("manager").bind(_manager);
    data.getReference("cache").bind(_cache);
  }

  BOSS_REGISTER_CLASS(ManifoldVoronoiExtractor);
}
