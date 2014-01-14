#ifndef VORONOI_EDGE_H
#define VORONOI_EDGE_H


#include <Eigen/Core>
#include <set>


class VoronoiVertex;

class VoronoiEdge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiEdge();
    VoronoiEdge(VoronoiVertex* from_, VoronoiVertex* to_);

    inline void setFrom(VoronoiVertex* from_) { _from = from_; }
    inline VoronoiVertex* from() { return _from; }
    inline const VoronoiVertex* from() const { return _from; }

    inline void setTo(VoronoiVertex* to_) { _to = to_; }
    inline VoronoiVertex* to() { return _to; }
    inline const VoronoiVertex* to() const { return _to; }

private:
    VoronoiVertex* _from;
    VoronoiVertex* _to;
};


struct EdgeComparator
{
    bool operator() (const VoronoiEdge& lhs, const VoronoiEdge& rhs) const;
};
typedef std::set<VoronoiEdge, EdgeComparator> EdgeSet;
#endif // VORONOI_EDGE_H
