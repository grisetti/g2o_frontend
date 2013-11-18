#ifndef VORONOI_VERTEX_H
#define VORONOI_VERTEX_H

#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include "voronoi_edge.h"


class VoronoiVertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiVertex();
    VoronoiVertex(float, Eigen::Vector2i);
    VoronoiVertex(float, int, Eigen::Vector2i);
    VoronoiVertex(const Eigen::Vector2i& par_, const Eigen::Vector2i& pos_, const float& dis_, const int& val_);

    ~VoronoiVertex();

    inline void addToEdgeSet(const VoronoiEdge& edge_) { _edgeSet.insert(edge_); }
    inline void setEdgeSet(EdgeSet edgeSet_) { _edgeSet = edgeSet_; }
    inline EdgeSet& edgeSet() { return _edgeSet;}
    inline const EdgeSet& edgeSet() const { return _edgeSet;}

    inline void setPosition(Eigen::Vector2i position_) { _position = position_; }
    inline void setPosition(int x, int y) { _position.x() = x; _position.y() = y; }
    inline Eigen::Vector2i& position() { return _position; }
    inline const Eigen::Vector2i& position() const { return _position; }

    inline void setDistance(float distance_) { _distance = distance_; }
    inline float distance() { return _distance; }
    inline float distance() const { return _distance; }

    inline void setVisited(bool visited_) { _visited = visited_;}
    inline bool visited() { return _visited; }
    inline bool visited() const { return _visited; }

    inline void setOrder(int order_) { _order = order_; }
    inline int order() { return _order; }
    inline int order() const { return _order; }

    inline void setProxy(Eigen::MatrixXf* proxy_) { _proxy = proxy_; }
    inline Eigen::MatrixXf* proxy() { return _proxy; }
    inline const Eigen::MatrixXf* proxy() const { return _proxy; }

    Eigen::Vector2i _parent;
    Eigen::Vector2i _position;

    Eigen::MatrixXf* _proxy;
    EdgeSet _edgeSet;

    float _distance;
    int _value;
    int _order;
    bool _visited;
};


struct VertexComparator
{
    inline bool operator() (const VoronoiVertex* lhs, const VoronoiVertex* rhs) const
    {
        if(lhs->_distance <= rhs->_distance)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
};

typedef Eigen::Matrix<VoronoiVertex, Eigen::Dynamic, Eigen::Dynamic> DistanceMap;
typedef std::priority_queue<VoronoiVertex*, std::vector<VoronoiVertex*>, VertexComparator> DistanceQueue;


struct Comparator
{
    inline bool operator() (const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) const
    {
        if(lhs.x() < rhs.x())
        {
            return true;
        }
        else if((lhs.x() == rhs.x()) && (lhs.y() < rhs.y()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};


typedef std::deque<VoronoiVertex*> PointersDeque;
typedef std::map<Eigen::Vector2i, VoronoiVertex, Comparator> VertexMap;
typedef std::pair<Eigen::Vector2i, VoronoiVertex> VoronoiPair;


#endif // VORONOI_VERTEX_H
