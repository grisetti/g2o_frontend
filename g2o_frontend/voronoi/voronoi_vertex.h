#ifndef VORONOI_VERTEX_H
#define VORONOI_VERTEX_H

#include <deque>
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
    VoronoiVertex(double, const Eigen::Vector2i&);
    VoronoiVertex(double, int, const Eigen::Vector2i&);
    VoronoiVertex(const Eigen::Vector2i& par_, const Eigen::Vector2i& pos_, const double& dis_, const int& val_);

    ~VoronoiVertex();

    inline void addToEdgeSet(const VoronoiEdge& edge_) { _edgeSet.insert(edge_); }
    void addToEdgeSet(const EdgeSet& es_);
    inline EdgeSet& edgeSet() { return _edgeSet;}
    inline const EdgeSet& edgeSet() const { return _edgeSet;}

    inline void setPosition(const Eigen::Vector2i& position_) { _position = position_; }
    inline void setPosition(int x, int y) { _position.x() = x; _position.y() = y; }
    inline Eigen::Vector2i& position() { return _position; }
    inline const Eigen::Vector2i& position() const { return _position; }

    inline void setDistance(double distance_) { _distance = distance_; }
    inline float distance() { return _distance; }
    inline float distance() const { return _distance; }

    inline void setElevation(double elevation_) { _elevation = elevation_; }
    inline float elevation() { return _elevation; }
    inline float elevation() const { return _elevation; }

    inline void setMerged() { _merged = true; }
    inline void setUnmerged() { _merged = false; }
    inline bool merged() { return _merged; }
    inline bool merged() const { return _merged; }

    inline void setPushed() { _pushed = true; }
    inline void setUnpushed() { _pushed = false; }
    inline bool pushed() { return _pushed; }
    inline bool pushed() const { return _pushed; }

    inline void setVisited() { _visited = true; }
    inline void setUnvisited() { _visited = false; }
    inline bool visited() { return _visited; }
    inline bool visited() const { return _visited; }

    inline void setNearest(const Eigen::Vector2i& nearest_) { _nearest = nearest_; }
    inline void setNearest(int x, int y) { _nearest.x() = x; _nearest.y() = y; }
    inline Eigen::Vector2i& nearest() { return _nearest;}
    inline const Eigen::Vector2i& nearest() const{ return _nearest;}

    inline void setParent(const Eigen::Vector2i& parent_) { _parent = parent_; }
    inline void setParent(int x, int y) { _parent.x() = x; _parent.y() = y; }
    inline Eigen::Vector2i& parent() { return _parent;}
    inline const Eigen::Vector2i& parent() const{ return _parent;}

    inline void setOrder(int order_) { _order = order_; }
    inline int order() { return _order; }
    inline int order() const { return _order; }

    inline void setValue(int value_) { _value = value_; }
    inline int value() { return _value; }
    inline int value() const { return _value; }

    /** Used when working with polar profiles. WILL BE DELETED*/
//    inline void setProxy(Eigen::MatrixXf* proxy_) { _proxy = proxy_; }
//    inline Eigen::MatrixXf* proxy() { return _proxy; }
//    inline const Eigen::MatrixXf* proxy() const { return _proxy; }


protected:
    Eigen::Vector2i _nearest;
    Eigen::Vector2i _parent;
    Eigen::Vector2i _position;

    /** Used when working with polar profiles. WILL BE DELETED*/
//    Eigen::MatrixXf* _proxy;
    EdgeSet _edgeSet;

    double _elevation;
    double _distance;
    int _value;
    int _order;
    bool _merged;
    bool _visited;
    bool _pushed;
};


//struct VertexComparator
//{
//    inline bool operator() (const VoronoiVertex* lhs, const VoronoiVertex* rhs) const
//    {
//        if(lhs->distance() <= rhs->distance())
//        {
//            return false;
//        }
//        else
//        {
//            return true;
//        }
//    }
//};


struct VertexComparator
{
    inline bool operator() (const VoronoiVertex* lhs, const VoronoiVertex* rhs) const
    {
        if(lhs->distance() < rhs->distance())
        {
            return false;
        }
        else if((lhs->distance() == rhs->distance()) && (lhs->position().x() < rhs->position().x()))
        {
            return false;
        }
        else if((lhs->distance() == rhs->distance()) && (lhs->position().x() == rhs->position().x()) && (lhs->position().y() < rhs->position().y()))
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
typedef std::priority_queue<VoronoiVertex*, std::vector<VoronoiVertex*>, VertexComparator> VoronoiQueue;
//typedef std::deque<VoronoiVertex*, VertexComparator> VoronoiDeque;


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


struct DistanceComparator
{
    inline bool operator() (const double& lhs, const double& rhs) const
    {
        if(lhs < rhs)
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
typedef std::map<Eigen::Vector2i, VoronoiVertex*, Comparator> VertexMap;
//typedef std::multimap<double, VoronoiVertex*, DistanceComparator> MinMaxMap;
typedef std::pair<Eigen::Vector2i, VoronoiVertex*> VoronoiPair;
//typedef std::pair<double, VoronoiVertex*> DistancePair;


#endif // VORONOI_VERTEX_H
