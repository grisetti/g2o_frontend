#ifndef GRAPH_MATCHER_H
#define GRAPH_MATCHER_H

#include <cstdlib>
#include <cmath>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <map>
#include <vector>
#include <set>
#include "utility.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o_frontend/mapper/matcher/matching/scan_matcher.h"


struct Information
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Information()
    {
        _transform.setIdentity();
        _parent = 0;
    }

    Eigen::Isometry2d _transform;
    g2o::VertexSE2* _parent;
};
typedef std::set<g2o::VertexSE2*> NodeSet;
typedef std::set<g2o::EdgeSE2*> EdgeSet;
typedef std::map<g2o::VertexSE2*, Information, std::less<g2o::VertexSE2*>,
        Eigen::aligned_allocator<std::pair<const g2o::VertexSE2*, Information> > > VertexInfoMap;


struct NodeMatcher
{
    virtual float match(g2o::SE2& result, g2o::VertexSE2* v1, g2o::VertexSE2* v2) = 0;
    virtual ~NodeMatcher();
};


struct IdealNodeMatcher : public NodeMatcher
{
    virtual float match(g2o::SE2& result, g2o::VertexSE2* v1, g2o::VertexSE2* v2);
    g2o::EdgeSE2* findEdge(g2o::VertexSE2* v1, g2o::VertexSE2* v2);
    EdgeSet _eset;
};


struct RealNodeMatcher : public NodeMatcher
{
    RealNodeMatcher(match_this::ScanMatcher* sm, const float& max);

    virtual float match(g2o::SE2& result, g2o::VertexSE2* v1, g2o::VertexSE2* v2);

    match_this::ScanMatcher* _smatcher;
    EdgeSet _eset;
    float _mscore;
};


struct GraphMatcher
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphMatcher();
    GraphMatcher(NodeMatcher* m);

    EdgeSet& results() { return _results; }
    NodeSet findNeighbors(g2o::HyperGraph::VertexIDMap* ref, const Eigen::Isometry2d& transform, double epsilon);
    void match(g2o::HyperGraph::VertexIDMap* ref, g2o::VertexSE2* first, const double& epsilon, const int& cnt);

    VertexInfoMap _currentInfo;
    EdgeSet _results;
    NodeMatcher* _matcher;
};


#endif // GRAPH_MATCHER_H
