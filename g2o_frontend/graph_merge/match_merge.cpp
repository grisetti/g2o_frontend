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

#include "graph_matcher.h"


using namespace Eigen;
using namespace g2o;
using namespace std;



int main(int argc, char** argv)
{
    int samples = 1500;
    char* filename = argv[argc-1];

    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    SparseOptimizer merged;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
    merged.setAlgorithm(solver);
    merged.load(filename);

    SparseOptimizer output;
    SlamLinearSolver* linearSolver1 = new SlamLinearSolver();
    linearSolver1->setBlockOrdering(false);
    SlamBlockSolver* blockSolver1 = new SlamBlockSolver(linearSolver1);
    OptimizationAlgorithmGaussNewton* solver1 = new OptimizationAlgorithmGaussNewton(blockSolver1);
    output.setAlgorithm(solver1);


    SparseOptimizer::VertexIDMap merged_vertices = merged.vertices();

    SparseOptimizer::VertexIDMap ref_vertices;
    SparseOptimizer::VertexIDMap curr_vertices;

    GraphMatcher vm;
    IdealNodeMatcher* im = new IdealNodeMatcher;

    int offset = samples + 10000;

    // Load the vertices in the graphs
    for(SparseOptimizer::VertexIDMap::iterator it = merged_vertices.begin(); it != merged_vertices.end(); it++)
    {

        VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
        VertexSE2* copy = new VertexSE2;
        copy->setId(v->id());
        copy->setEstimate(v->estimate());
        output.addVertex(copy);

        Information info;
        info._parent = NULL;
        info._transform = copy->estimate().toIsometry();
        vm._currentInfo.insert(make_pair(copy, info));

        if(v->id() < offset)
        {
            ref_vertices.insert(make_pair(copy->id(), copy));
        }
        else
        {
            curr_vertices.insert(make_pair(copy->id(), copy));
        }

    }

    SparseOptimizer::EdgeSet& edges = merged.edges();
    cout << "Adding edges" << endl;

    for(SparseOptimizer::EdgeSet::iterator it = edges.begin(); it != edges.end(); it++)
    {
        EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
        EdgeSE2* newEdge = new EdgeSE2;
        VertexSE2* v0New = (VertexSE2*)output.vertex(e->vertex(0)->id());
        VertexSE2* v1New = (VertexSE2*)output.vertex(e->vertex(1)->id());
        newEdge->setVertex(0,v0New);
        newEdge->setVertex(1,v1New);
        newEdge->setMeasurement(e->measurement());
        newEdge->setInformation(e->information());


        if(v0New->id() >= offset && v1New->id() < offset)
        {
            im->_eset.insert(newEdge);
        } else {
            output.addEdge(newEdge);
        }
    }

    cerr << "loaded " << endl;
    cerr<<  "       " << output.vertices().size() << " vertices" << endl;
    cerr << "       " << output.edges().size() << " edges" << endl;
    cerr << "       " << im->_eset.size() << " closures" << endl;
    vm._matcher = im;

    cout << "Ref_vert size: " << ref_vertices.size() << endl;
    cout << "Curr_vert size: " << curr_vertices.size() << endl;
    VertexSE2* first = dynamic_cast<VertexSE2*>(curr_vertices.find(offset)->second);
    vm.match(&ref_vertices, first, 2.0);

    EdgeSet matches = vm.results();
    for(EdgeSet::iterator it = matches.begin(); it != matches.end(); it++)
    {
        EdgeSE2* e = *it;
        output.addEdge(e);
    }

    ostringstream result;
    result << "simulated_result.g2o";
    output.save(result.str().c_str());

    cout << "Finished" << endl;
    return 0;
}
