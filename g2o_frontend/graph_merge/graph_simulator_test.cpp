#include <iostream>
#include <cmath>

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "graph_simulator.h"


using namespace Eigen;
using namespace g2o;
using namespace std;



int main()
{
    int samples = 1500;
    GraphSimulator gs;
    gs.simulate(samples);

    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    SparseOptimizer optimizer;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

    optimizer.setAlgorithm(solver);

    const SimGraph& traj = gs.trajectories()[0];
    for(size_t i = 0; i < traj.poses().size(); ++i)
    {
        const SimNode& pose = traj.poses()[i];
        const Isometry2d& transform = pose.noisy_pose;
        VertexSE2* robot = new VertexSE2;
        robot->setId(pose.id);
        robot->setEstimate(transform);
        optimizer.addVertex(robot);
    }

    for(Edges::iterator it = traj.edges().begin(); it != traj.edges().end(); ++it)
    {
        SimEdge* edge = *it;

        EdgeSE2* odometry = new EdgeSE2;
        odometry->vertices()[0] = optimizer.vertex(edge->from_id);
        odometry->vertices()[1] = optimizer.vertex(edge->to_id);
        odometry->setMeasurement(edge->noisy_transform);
        odometry->setInformation(edge->information);
        optimizer.addEdge(odometry);
    }

    ostringstream before;
    before << "pre_optimization.g2o";
    optimizer.save(before.str().c_str());

    VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);
    optimizer.setVerbose(true);

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    ostringstream after;
    after << "post_optimization.g2o";
    optimizer.save(after.str().c_str());

    optimizer.clear();

    Factory::destroy();
    OptimizationAlgorithmFactory::destroy();
    HyperGraphActionLibrary::destroy();

    return 0;
}
