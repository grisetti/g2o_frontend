#include "voronoi_vertex.h"

#define INF 1000000

using namespace std;
using namespace Eigen;


VoronoiVertex::VoronoiVertex()
{
    _merged = false;
    _visited = false;

    _nearest = Eigen::Vector2i(INF, INF);
    _parent = Eigen::Vector2i(INF, INF);
    _position = Eigen::Vector2i(INF, INF);
    _value = 0;
    _distance = 0.0;
    _pushed = false;
}


VoronoiVertex::VoronoiVertex(double& distance_, const Eigen::Vector2i& position_)
{
    _distance = distance_;
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


bool VoronoiVertex::write(ostream& os)
{
    os << _id << " " << _position.x() << " " << _position.y() << " " << 0;
    return os.good();
}


bool VoronoiData::write(ostream& os)
{
    return os.good();
}
