#include "voronoi_vertex.h"

#define INF 1000000

using namespace std;
using namespace Eigen;


VoronoiVertex::VoronoiVertex()
{
    _order = 0;
    _merged = false;
    _visited = false;

    _nearest = Eigen::Vector2i(INF, INF);
    _parent = Eigen::Vector2i(INF, INF);
    _position = Eigen::Vector2i(INF, INF);
    _value = 0;
    _distance = 0.0;
    _pushed = false;
}


VoronoiVertex::VoronoiVertex(double distance_, const Eigen::Vector2i& position_)
{
    _distance = distance_;
    _order = 0;
    _position = position_;
    _merged = false;
    _visited = false;
    _pushed = false;
}


VoronoiVertex::VoronoiVertex(double distance_, int order_, const Eigen::Vector2i& position_)
{
    _distance = distance_;
    _order = order_;
    _position = position_;
    _merged = false;
    _visited = false;
    _pushed = false;
}


VoronoiVertex::VoronoiVertex(const Eigen::Vector2i& par_, const Eigen::Vector2i& pos_, const double& dis_, const int& val_)
{
    _parent = par_;
    _position = pos_;
    _distance = dis_;
    _value = val_;

    _merged = false;
    _visited = false;
    _pushed = false;
}


VoronoiVertex::~VoronoiVertex() {;}


void VoronoiVertex::addToEdgeSet(const EdgeSet& es_)
{
    for(EdgeSet::iterator it = es_.begin(); it != es_.end(); ++it)
    {
        VoronoiEdge e = *it;
        e.setFrom(this);
        _edgeSet.insert(e);
    }
}

