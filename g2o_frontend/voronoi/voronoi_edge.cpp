#include "voronoi_edge.h"
#include "voronoi_vertex.h"


using namespace std;
using namespace Eigen;


VoronoiEdge::VoronoiEdge()
{
    _to = 0;
}


VoronoiEdge::VoronoiEdge(VoronoiVertex* from_, VoronoiVertex* to_)
{
    _from = from_;
    _to = to_;
}


bool VoronoiEdge::write(ostream& os)
{
    Rotation2Dd r(0);
    r.fromRotationMatrix(_transform.linear());
    Vector2d t = _transform.translation();

    os << _from->id() << " " << _to->id();
    os << " " << t.x() << " " << t.y() << " " << r.angle();
    os << " 1000 0 0 1000 0 1000";
    return os.good();
}


bool EdgeComparator::operator()(const VoronoiEdge* lhs, const VoronoiEdge* rhs) const
{
    const VoronoiVertex* lhs_from = lhs->from();
    const int lhs_from_x = lhs_from->position().x();
    const int lhs_from_y = lhs_from->position().y();

    const VoronoiVertex* lhs_to = lhs->to();
    const int lhs_to_x = lhs_to->position().x();
    const int lhs_to_y = lhs_to->position().y();


    const VoronoiVertex* rhs_from = rhs->from();
    const int rhs_from_x = rhs_from->position().x();
    const int rhs_from_y = rhs_from->position().y();

    const VoronoiVertex* rhs_to = rhs->to();
    const int rhs_to_x = rhs_to->position().x();
    const int rhs_to_y = rhs_to->position().y();


    if((lhs_from->position() == rhs_from->position()) && (lhs_to_x == rhs_to_x) && (lhs_to_y < rhs_to_y))
    {
        return true;
    }
    else if((lhs_from->position() == rhs_from->position()) && (lhs_to_x < rhs_to_x))
    {
        return true;
    }
    else if((lhs_from_x == rhs_from_x) && (lhs_from_y < rhs_from_y))
    {
        return true;
    }
    else if(lhs_from_x < rhs_from_x)
    {
        return true;
    }
    else
    {
        return false;
    }
}
