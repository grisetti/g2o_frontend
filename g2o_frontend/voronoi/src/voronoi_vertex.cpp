#include "voronoi_vertex.h"

#define INF 1000000

using namespace std;
using namespace Eigen;


VoronoiVertex::VoronoiVertex()
{
    _order = 0;
    _visited = false;

    _parent = Eigen::Vector2i(INF, INF);
    _position = Eigen::Vector2i(INF, INF);
    _value = 0;
    _distance = 0.0;

    _proxy = 0;
}


VoronoiVertex::VoronoiVertex(float distance_, Vector2i position_)
{
    _distance = distance_;
    _order = 0;
    _position = position_;
    _visited = false;

    _proxy = 0;
}


VoronoiVertex::VoronoiVertex(float distance_, int order_, Vector2i position_)
{
    _distance = distance_;
    _order = order_;
    _position = position_;
    _visited = false;

    _proxy = 0;
}


VoronoiVertex::VoronoiVertex(const Eigen::Vector2i& par_, const Eigen::Vector2i& pos_, const float& dis_, const int& val_)
{
    _parent = par_;
    _position = pos_;
    _distance = dis_;
    _value = val_;

    _proxy = 0;
}


VoronoiVertex::~VoronoiVertex() {;}
