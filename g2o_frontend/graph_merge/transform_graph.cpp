#include <vector>

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/data/robot_laser.h"
#include "g2o/types/data/vertex_tag.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#define MARKUSED(x) x=x


using namespace Eigen;
using namespace g2o;
using namespace std;


typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;


int main(int argc, char** argv)
{
    char* input_filename = argv[argc-2];
    int reference_id = atoi(argv[argc-1]);

    SparseOptimizer input;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(blockSolver);
    input.setAlgorithm(solver);
    input.load(input_filename);

    SparseOptimizer output;
    SlamLinearSolver* linearSolver2 = new SlamLinearSolver();
    linearSolver2->setBlockOrdering(false);
    SlamBlockSolver* blockSolver2 = new SlamBlockSolver(linearSolver2);
    OptimizationAlgorithmLevenberg* solver2 = new OptimizationAlgorithmLevenberg(blockSolver2);
    output.setAlgorithm(solver2);

    SparseOptimizer::VertexIDMap vertices = input.vertices();
    SparseOptimizer::EdgeSet edges = input.edges();

    if(vertices.find(reference_id) == vertices.end())
    {
        cout << "Vertex not found, exiting ..." << endl;
        return -1;
    }

    VertexSE2* reference_vertex = dynamic_cast<VertexSE2*>(vertices.find(reference_id)->second);
    SE2 iref = reference_vertex->estimate().inverse();

    for(SparseOptimizer::VertexIDMap::iterator it = vertices.begin(); it != vertices.end(); it++)
    {
        VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);

        VertexSE2* copy = new VertexSE2;
        copy->setId(v->id());
        copy->setEstimate(iref * v->estimate());
        OptimizableGraph::Data* d = v->userData();
        bool addTag = true;
        while(d)
        {
            const VertexTag* vtag = dynamic_cast<const VertexTag*>(d);
            if(vtag)
            {
                VertexTag* copytag = new VertexTag;
                stringstream ss;
                ss << copy->id();
                copytag->setName(ss.str());
                copy->addUserData(copytag);
                addTag = false;
            }

            const RobotLaser* vdata = dynamic_cast<const RobotLaser*>(d);
            if(vdata)
            {
                RobotLaser* copydata = new RobotLaser;
                copydata->setRanges(vdata->ranges());
                copydata->setRemissions(vdata->remissions());
                copydata->setLaserParams(vdata->laserParams());
                copydata->setOdomPose(vdata->odomPose());
                copy->setUserData(copydata);
            }
            d = d->next();
        }
        if(addTag)
        {
            VertexTag* copytag = new VertexTag;
            stringstream ss;
            ss << copy->id();
            copytag->setName(ss.str());
            copy->addUserData(copytag);
        }
        output.addVertex(copy);
    }

    for(SparseOptimizer::EdgeSet::iterator it = edges.begin(); it != edges.end(); it++)
    {
        EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
        EdgeSE2* newEdge = new EdgeSE2;
        VertexSE2* v0New = (VertexSE2*) output.vertex(e->vertex(0)->id());
        VertexSE2* v1New = (VertexSE2*) output.vertex(e->vertex(1)->id());
        newEdge->setVertex(0,v0New);
        newEdge->setVertex(1,v1New);
        newEdge->setMeasurement(e->measurement());
        newEdge->setInformation(e->information());

        output.addEdge(newEdge);
    }

    ostringstream oss;
    oss << "tranformed_" << input_filename;
    output.save(oss.str().c_str());
}
