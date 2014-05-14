#ifndef VORONOI_VERTEX_H
#define VORONOI_VERTEX_H

#include <ctime>
#include <deque>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include "voronoi_edge.h"



class VoronoiData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiData()
    {
        _sensorPose.setIdentity();
    }

    inline void setPose(const Eigen::Isometry2d& sp) { _sensorPose = sp; }
    virtual bool write(std::ostream& os) = 0;

    Eigen::Isometry2d _sensorPose;
    std::string _tag;
};


class VoronoiLaser : public VoronoiData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiLaser();

    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

    inline const std::vector<double>& ranges() const { return _ranges;}
    inline void setRanges(const std::vector<double>& ranges) { _ranges = ranges; }

    virtual bool write(std::ostream& os);

    inline double& maxRange() { return _maxRange; }
    inline double& angularStep() { return _angularStep; }
    inline double& firstBeamAngle() { return _firstBeamAngle; }
    inline double& fov() { return _fov; }

protected:
    std::vector<double> _ranges;

    int _type;
    double _firstBeamAngle;
    double _fov;
    double _angularStep;
    double _accuracy;
    int _remissionMode;
    double _maxRange;
};


class VoronoiVertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoronoiVertex();
    VoronoiVertex(const VoronoiVertex* v);

    ~VoronoiVertex();

    inline void setId(int id) { _id = id; }
    inline int& id() { return _id; }
    inline const int& id() const { return _id; }

    inline void addToEdgeSet(VoronoiEdge* edge_) { _edgeSet.insert(edge_); }
    inline EdgeSet& edgeSet() { return _edgeSet;}
    inline const EdgeSet& edgeSet() const { return _edgeSet;}

    inline void setPosition(const Eigen::Vector2i& position_) { _position = position_; }
    inline void setPosition(int x, int y) { _position.x() = x; _position.y() = y; }
    inline Eigen::Vector2i& position() { return _position; }
    inline const Eigen::Vector2i& position() const { return _position; }

    inline void setComponent(double component_) { _component = component_; }
    inline float component() { return _component; }
    inline float component() const { return _component; }

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

    inline void setValue(int value_) { _value = value_; }
    inline int value() { return _value; }
    inline int value() const { return _value; }

    bool write(std::ostream& os);

    inline void setData(VoronoiData* d) { _data = d; }
    inline VoronoiData* data() { return _data; }

    inline void setGraphPose(const Eigen::Vector3d& gp) { _graphPose = gp; }
    inline Eigen::Vector3d& graphPose() { return _graphPose; }
    inline const Eigen::Vector3d& graphPose() const { return _graphPose; }

    inline Eigen::Isometry2d toIsometry() const
    {
        Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
        iso.linear() = Eigen::Rotation2Dd(_graphPose.z()).toRotationMatrix();
        iso.translation() = Eigen::Vector2d(_graphPose.x(), _graphPose.y());
        return iso;
    }

protected:
    Eigen::Vector2i _nearest;
    Eigen::Vector2i _parent;
    Eigen::Vector2i _position;
    Eigen::Vector3d _graphPose;

    EdgeSet _edgeSet;

    double _elevation;
    double _distance;
    int _value;
    int _id;
    int _component;
    bool _merged;
    bool _visited;
    bool _pushed;

    VoronoiData* _data;
};


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


struct Comparator
{
    inline bool operator() (const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) const
    {
        if(lhs.y() < rhs.y())
        {
            return true;
        }
        else if((lhs.y() == rhs.y()) && (lhs.x() < rhs.x()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

//struct Comparator
//{
//    inline bool operator() (const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) const
//    {
//        if(lhs.x() < rhs.x())
//        {
//            return true;
//        }
//        else if((lhs.x() == rhs.x()) && (lhs.y() < rhs.y()))
//        {
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//};


struct IDCompare
{
    inline bool operator() (const VoronoiVertex* lhs, const VoronoiVertex* rhs) const
    {
        if(lhs->id() < rhs->id()) return true;
        else return false;
    }
};

typedef std::map<Eigen::Vector2i, VoronoiVertex*, Comparator> VertexMap;
#endif // VORONOI_VERTEX_H
