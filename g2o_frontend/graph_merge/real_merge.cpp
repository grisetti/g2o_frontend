#include <iostream>
#include <cmath>

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

#include "graph_matcher.h"
#include "g2o_frontend/mapper/matcher/matching/correlative_matcher.h"


#define MARKUSED(x) x = x


using namespace Eigen;
using namespace g2o;
using namespace match_this;
using namespace std;


int main(int argc, char** argv)
{
    if(argc != 5)
    {
        cerr << "Missing arguments." << endl;
        cerr << "Expected: file_merge <input1>.g2o <input2>.g2o <output>.g2o offset_id" << endl;
        return -1;
    }

    char* ref_filename = argv[1];
    char* curr_filename = argv[2];
    char* out_filename = argv[3];
    int reference_id = atoi(argv[argc-1]);

    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    SparseOptimizer input1;
    SlamLinearSolver* linearSolver1 = new SlamLinearSolver();
    linearSolver1->setBlockOrdering(false);
    SlamBlockSolver* blockSolver1 = new SlamBlockSolver(linearSolver1);
    OptimizationAlgorithmLevenberg* solver1 = new OptimizationAlgorithmLevenberg(blockSolver1);
    input1.setAlgorithm(solver1);
    input1.load(ref_filename);

    SparseOptimizer input2;
    SlamLinearSolver* linearSolver2 = new SlamLinearSolver();
    linearSolver2->setBlockOrdering(false);
    SlamBlockSolver* blockSolver2 = new SlamBlockSolver(linearSolver2);
    OptimizationAlgorithmLevenberg* solver2 = new OptimizationAlgorithmLevenberg(blockSolver2);
    input2.setAlgorithm(solver2);
    input2.load(curr_filename);

    SparseOptimizer output;
    SlamLinearSolver* linearSolver3 = new SlamLinearSolver();
    linearSolver2->setBlockOrdering(false);
    SlamBlockSolver* blockSolver3 = new SlamBlockSolver(linearSolver3);
    OptimizationAlgorithmLevenberg* solver3 = new OptimizationAlgorithmLevenberg(blockSolver3);
    output.setAlgorithm(solver3);

    float resolution = 0.03;
    float kernelMaxValue = 1;
    float radius = 100;
    CorrelativeMatcher* cm = new CorrelativeMatcher(resolution, radius, kernelMaxValue, kernelMaxValue);

    const float maxScore = 100; // It was 250
    RealNodeMatcher* rm = new RealNodeMatcher(cm, maxScore);
    GraphMatcher* gm = new GraphMatcher(rm);

    SparseOptimizer::VertexIDMap ref_vertices;
    SparseOptimizer::VertexIDMap curr_vertices;
    for(SparseOptimizer::VertexIDMap::iterator it = input1.vertices().begin(); it != input1.vertices().end(); it++)
    {
        VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);

        VertexSE2* copy = new VertexSE2;
        copy->setId(v->id());
        copy->setEstimate(v->estimate());
        OptimizableGraph::Data* d = v->userData();
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
            }

            const RobotLaser* vdata = dynamic_cast<const RobotLaser*>(d);
            if(vdata)
            {
                RobotLaser* copydata = new RobotLaser;
                copydata->setRanges(vdata->ranges());
                copydata->setRemissions(vdata->remissions());
                copydata->setLaserParams(vdata->laserParams());
                copydata->setOdomPose(vdata->odomPose());
                copy->addUserData(copydata);
            }
            d = d->next();
        }
        output.addVertex(copy);

        Information info;
        info._parent = NULL;
        info._transform = copy->estimate().toIsometry();
        gm->_currentInfo.insert(make_pair(copy, info));
        ref_vertices.insert(make_pair(copy->id(), copy));
    }

    for(SparseOptimizer::EdgeSet::iterator it = input1.edges().begin(); it != input1.edges().end(); it++)
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

//    int offset = input1.vertices().size() + 10000;
    int offset = 100000;
    for(SparseOptimizer::VertexIDMap::iterator it = input2.vertices().begin(); it != input2.vertices().end(); it++)
    {
        VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
        VertexSE2* copy = new VertexSE2;
        copy->setId(v->id() + offset);
        copy->setEstimate(v->estimate());
        OptimizableGraph::Data* d = v->userData();
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
            }

            const RobotLaser* vdata = dynamic_cast<const RobotLaser*>(d);
            if(vdata)
            {
                RobotLaser* copydata = new RobotLaser;
                copydata->setRanges(vdata->ranges());
                copydata->setRemissions(vdata->remissions());
                copydata->setLaserParams(vdata->laserParams());
                copydata->setOdomPose(vdata->odomPose());
                copy->addUserData(copydata);
            }
            d = d->next();
        }
        output.addVertex(copy);

        Information info;
        info._parent = NULL;
        info._transform = copy->estimate().toIsometry();
        gm->_currentInfo.insert(make_pair(copy, info));
        curr_vertices.insert(make_pair(copy->id(), copy));
    }

    for(SparseOptimizer::EdgeSet::iterator it = input2.edges().begin(); it != input2.edges().end(); it++)
    {
        EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
        EdgeSE2* newEdge = new EdgeSE2;

        int new_from = e->vertex(0)->id() + offset;
        int new_to = e->vertex(1)->id() + offset;

        VertexSE2* v0New = (VertexSE2*) output.vertex(new_from);
        VertexSE2* v1New = (VertexSE2*) output.vertex(new_to);
        newEdge->setVertex(0,v0New);
        newEdge->setVertex(1,v1New);
        newEdge->setMeasurement(e->measurement());
        newEdge->setInformation(e->information());

        output.addEdge(newEdge);
    }

    VertexSE2* first = dynamic_cast<VertexSE2*>(curr_vertices.find(offset + reference_id)->second);
    gm->match(&ref_vertices, first, 1, -1);

    EdgeSet res=gm->results();
    for (EdgeSet::iterator it=res.begin(); it!=res.end(); it++)
    {
        output.addEdge(*it);
    }

    output.save(out_filename);

    VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(output.vertex(0));
    firstRobotPose->setFixed(true);
    output.setVerbose(true);

    output.initializeOptimization();
    output.optimize(10);

    ofstream os1("optimized_subset1.g2o");
    ofstream os2("optimized_subset2.g2o");

    OptimizableGraph::VertexSet vset1, vset2;
    for(OptimizableGraph::VertexIDMap::const_iterator it = output.vertices().begin(); it != output.vertices().end(); it++)
    {
        VertexSE2* v = (VertexSE2*) it->second;
        if(v->id() < offset)
        {
            vset1.insert(v);
        }
        else
        {
            v->setId(v->id()-offset);
            vset2.insert(v);
        }
    }

    OptimizableGraph::EdgeSet eset1, eset2;
    for(SparseOptimizer::EdgeSet::iterator it = output.edges().begin(); it != output.edges().end(); it++)
    {
        EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
        if(e->vertex(0)->id() < offset && e->vertex(1)->id() < offset)
        {
            eset1.insert(e);
        }
        else if(e->vertex(0)->id() >= offset && e->vertex(1)->id() >= offset)
        {
            eset2.insert(e);
        }
    }

    output.saveSubset(os1, vset1);
    output.saveSubset(os2, vset2);

    return 0;
}
