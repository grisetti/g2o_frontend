#pragma once
#include "pwn_tracker.h"
#include "pwn_closer.h"
#include <QGLViewer/qglviewer.h>
#include "GL/gl.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include "g2o_frontend/boss_map/stream_processor.h"
#include <list>


namespace manifold_voronoi
{
using namespace pwn;
using namespace boss_map;
using namespace boss_map_building;
using namespace pwn_tracker;


class ManifoldVoronoiData: public ImageData
{
public:
    ManifoldVoronoiData(int id=-1, IdContext* context = 0);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    float resolution;
    MapNode* node;
};


class ManifoldVoronoiExtractor: public StreamProcessor
{
public:
    ManifoldVoronoiExtractor(int id = -1, boss::IdContext* context = 0);
    void init();
    virtual void process(Serializable* s);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    void setResolution(float res) {_resolution = res;}
    float resolution() const {return _resolution;}
protected:
    std::list<PwnCloudCache::HandleType> cacheHandles;
    size_t _dequeSize;
    MapManager* _manager;
    PwnCloudCache* _cache;
    float _resolution;
    float _normalThreshold;
    int _xSize, _ySize;
};
}
