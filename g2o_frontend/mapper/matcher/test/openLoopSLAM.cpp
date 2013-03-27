#include <signal.h>

#include "g2o/core/hyper_dijkstra.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"

#include "g2o_frontend/mapper/matcher/matching/correlative_matcher.h"
#include "g2o_frontend/sensor_data/laser_robot_data.h"


using namespace Eigen;
using namespace g2o;
using namespace std;



VertexSE2* v = new VertexSE2;
EdgeSE2* e = new EdgeSE2;
LaserRobotData* lrd = new LaserRobotData;


volatile bool hasToStop;
void sigquit_handler(int sig)
{
    if(sig == SIGINT)
    {
        hasToStop = 1;
        static int cnt = 0;
        if(cnt++ == 2)
        {
            cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
            exit(1);
        }
    }
}


int main(int argc, char**argv)
{
    hasToStop = false;
    string filename;
    string outputFilename;
    CommandArgs arg;
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.paramLeftOver("graph-output", outputFilename , "", "output graph file", true);
    arg.parseArgs(argc, argv);

    // graph construction
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
    SparseOptimizer* graph = new SparseOptimizer();
    graph->setAlgorithm(solverGauss);
    graph->load(filename.c_str());


    // some parameters for the matcher
    float resolution = 0.03;
    float kernelMaxValue = 1;

    float radius = 100;
    CorrelativeMatcher cm(resolution, radius, kernelMaxValue, kernelMaxValue);
    LaserRobotData::Vector2fVector previousScan, previousReducedScan;
    LaserRobotData::Vector2fVector currentScan, currentReducedScan;

    // sort the vertices based on the id
    vector<int> vertexIds(graph->vertices().size());
    int k=0;
    for (OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it)
    {
        vertexIds[k++] = (it->first);
    }

    sort(vertexIds.begin(), vertexIds.end());

    Factory* factory = Factory::instance();
    bool firstVertex = true;
    VertexSE2* previousVertex = new VertexSE2;
    for(size_t i = 0; i < vertexIds.size() && ! hasToStop; ++i)
    {
        OptimizableGraph::Vertex* v = graph->vertex(vertexIds[i]);
        cerr << "vertex: " << v->id() << " type:" << factory->tag(v) << endl;
        OptimizableGraph::Data* d = v->userData();
        k = 0;
        while(d)
        {
            if(d)
            {
                cerr << "\t payload: " << factory->tag(d) << endl;
                const LaserRobotData* laserRobotData = dynamic_cast<const LaserRobotData*>(d);
                VertexSE2* vse2 = dynamic_cast<VertexSE2*>(v);
                if(laserRobotData && vse2)
                {
                    if(firstVertex)
                    {
                        previousScan = laserRobotData->floatCartesian();
                        firstVertex = false;
                        previousVertex = vse2;
                        previousVertex->setFixed(true);
                    }
                    else
                    {
                        if(previousVertex->id() != vse2->id())
                        {
                            currentScan = laserRobotData->floatCartesian();
                            Isometry2d delta = previousVertex->estimate().toIsometry().inverse() * vse2->estimate().toIsometry();
                            Matrix2f mat = delta.rotation().cast<float>();
                            float angle = atan2(mat(1, 0), mat(0, 0));
                            Vector3f initGuess(delta.translation().x(), delta.translation().y(), angle);
                            cout << "initial guess: " << initGuess.x() << ", " << initGuess.y() << ", " << initGuess.z() << endl;
                            Vector3f lower(-0.3+initGuess.x(), -0.3+initGuess.y(), -0.2+initGuess.z());
                            Vector3f upper(0.3+initGuess.x(), 0.3+initGuess.y(), 0.2+initGuess.z());
                            float thetaRes = 0.01;
                            int max = 350;

                            cm.convolveScan(previousScan);
                            cm.scanMatch(currentScan, lower, upper, thetaRes, max);
                            cm.clear();

                            Vector3f tsf = cm.getMatches()[0]->_transformation;
                            cout << "Current transformation: " << tsf.x() << ", " << tsf.y() << ", " << tsf.z() << endl;
                            cout << "Matches size: " << cm.getMatches().size() << endl;

                            cm.clearMatchResults();

                            EdgeSE2* match = new EdgeSE2;
                            match->setVertex(0, previousVertex);
                            match->setVertex(1, vse2);

                            match->setParameterId(0, laserRobotData->paramIndex());
                            match->setParameterId(1, laserRobotData->paramIndex());
                            Eigen::Isometry2d meas;
                            meas.setIdentity();
                            meas = Rotation2Dd(tsf.z());
                            meas.translation() = Vector2d(tsf.x(), tsf.y());
                            match->setMeasurement(meas);

                            Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 10;
                            info(2, 2) *= 1000;

                            match->setInformation(info);
                            bool res = graph->addEdge(match);

                            previousReducedScan = currentReducedScan;
                            previousScan = currentScan;
                            previousVertex = vse2;
                        }
                    }
                }
                k++;
            }
            d = d->next();
        }
    }


    if(outputFilename != "")
    {
        graph->save(outputFilename.c_str());
        cout << "Graph saved" << endl;
    }
    else
    {
        cout << "Output filename not provided" << endl;
    }
}
