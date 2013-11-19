#include "voronoi_edge.h"
#include "voronoi_vertex.h"


using namespace std;
using namespace Eigen;


VoronoiEdge::VoronoiEdge()
{
    _from = 0;
    _to = 0;
}


VoronoiEdge::VoronoiEdge(VoronoiVertex* from_, VoronoiVertex* to_)
{
    _from = from_;
    _to = to_;
}


bool EdgeComparator::operator()(const VoronoiEdge& lhs, const VoronoiEdge& rhs) const
{
    if((lhs.from()->position().x() < rhs.from()->position().x())
            || ((lhs.from()->position().x() == rhs.from()->position().x()) && (lhs.from()->position().y() < rhs.from()->position().y())))
    {
        return true;
    }
    else
    {
        return false;
    }
}
