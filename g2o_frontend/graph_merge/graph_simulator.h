#ifndef GRAPH_SIMULATOR_H
#define GRAPH_SIMULATOR_H

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


struct Noise
{
    static double gaussian(double mean, double sigma)
    {
        double x, y, r2;
        do
        {
            x = -1.0 + 2.0 * uniform(0.0, 1.0);
            y = -1.0 + 2.0 * uniform(0.0, 1.0);
            r2 = x * x + y * y;
        }
        while(r2 > 1.0 || r2 == 0.0);
        return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
    }

    static double uniform(double lower, double upper)
    {
        return lower + ((double) std::rand() / (RAND_MAX + 1.0)) * (upper - lower);
    }

    static void rand_seed()
    {
        rand_seed((uint) std::time(NULL));
    }

    static void rand_seed(uint s)
    {
        std::srand(s);
    }
};

struct SimEdge;
typedef std::set<SimEdge*> Edges;

struct SimNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const Edges& connections() const { return _connections; }

    int id;
    Eigen::Isometry2d real_pose;
    Eigen::Isometry2d noisy_pose;
    Edges _connections;
};
typedef std::vector<SimNode, Eigen::aligned_allocator<SimNode> > Poses;
typedef std::map<int, SimNode> NodeMap;

struct SimEdge
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SimEdge();
    ~SimEdge();

    int from_id;
    int to_id;

    Eigen::Isometry2d real_transform;
    Eigen::Isometry2d noisy_transform;
    Eigen::Matrix3d information;
};

struct SimGraph
{
    void vec2map();
    inline const NodeMap& nodeMap() const { return _nodes; }
    inline const Poses& poses() const { return _poses; }
    inline const Edges& edges() const { return _edges; }

    NodeMap _nodes;
    Poses _poses;
    Edges _edges;
};
typedef std::vector<SimGraph> Trajectories;


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
typedef std::map<g2o::VertexSE2*, Information, std::less<g2o::VertexSE2*>,
                 Eigen::aligned_allocator<std::pair<const g2o::VertexSE2*, Information> > > VertexInfoMap;


typedef std::set<g2o::VertexSE2*> NodeSet;
typedef std::set<g2o::EdgeSE2*> EdgeSet;


struct VirtualMatcher
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSet& results() { return _results; }
    NodeSet findNeighbors(g2o::HyperGraph::VertexIDMap* ref, const Eigen::Isometry2d& transform, double epsilon);
    void match(g2o::HyperGraph::VertexIDMap *ref, g2o::HyperGraph::VertexIDMap *curr, g2o::VertexSE2* first, double epsilon);
    void tryMatch(g2o::VertexSE2* neighbor, g2o::VertexSE2* node, double &score, Eigen::Isometry2d& tStar);

    std::set<SimEdge*> _matches;
    std::set<SimNode*> _visited1, _visited2;
    VertexInfoMap _currentInfo;
    EdgeSet _results;
};

typedef std::vector<int*> Prova;
struct GraphSimulator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum MotionType
    {
      LEFT,
      RIGHT,
      POSSIBLE_MOTIONS
    };

    static Eigen::Isometry2d addNoise(const Eigen::Isometry2d& lastPose, const Eigen::Vector3d& noise);

    void simulate(int samples, const Eigen::Isometry2d& offset = Eigen::Isometry2d::Identity());
    void simulate(int samples, int trajectories, bool interClosures = 0, bool lookForClosures = 1, const Eigen::Isometry2d& offset = Eigen::Isometry2d::Identity());

    const Trajectories& trajectories() const { return _trajectories; }
    const Edges& closures() const { return _closures; }

    Prova prova;
protected:
    Trajectories _trajectories;
    Edges _closures;

    Eigen::Isometry2d generateMotion(int dir, double step);
    SimNode generatePose(const SimNode& last, const Eigen::Isometry2d& motion, const Eigen::Vector3d& noise);
};
#endif // GRAPH_SIMULATOR_H
