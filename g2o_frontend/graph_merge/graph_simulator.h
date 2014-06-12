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

    inline const Edges& connections() const { return _connections; }

    int id;
    Eigen::Isometry2d real_pose;
    Eigen::Isometry2d noisy_pose;
    Edges _connections;
};
typedef std::vector<SimNode, Eigen::aligned_allocator<SimNode> > Poses;


struct SimEdge
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int from_id;
    int to_id;

    Eigen::Isometry2d real_transform;
    Eigen::Isometry2d noisy_transform;
    Eigen::Matrix3d information;
};


struct SimGraph
{
    inline const Poses& poses() const { return _poses; }
    inline const Edges& edges() const { return _edges; }

    Poses _poses;
    Edges _edges;
};
typedef std::vector<SimGraph> Trajectories;


struct GraphSimulator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphSimulator(const Eigen::Vector3d& noise = Eigen::Vector3d::Identity(), bool randomize=false);

    enum MotionType
    {
      LEFT,
      RIGHT,
      POSSIBLE_MOTIONS
    };

    Eigen::Isometry2d addNoise(const Eigen::Isometry2d& lastPose);

    void simulate(int samples, const Eigen::Isometry2d& offset = Eigen::Isometry2d::Identity());
    void simulate(int samples, int trajectories, bool interClosures = 0, bool lookForClosures = 1, const Eigen::Isometry2d& offset = Eigen::Isometry2d::Identity());

    inline const Trajectories& trajectories() const { return _trajectories; }
    inline const Edges& closures() const { return _closures; }

protected:
    Trajectories _trajectories;
    Edges _closures;

    Eigen::Vector3d _noise;
    Eigen::Isometry2d generateMotion(int dir, double step);
    SimNode generatePose(const SimNode& last, const Eigen::Isometry2d& motion);
};
#endif // GRAPH_SIMULATOR_H
