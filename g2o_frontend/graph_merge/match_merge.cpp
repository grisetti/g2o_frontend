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
    Information info;

    SparseOptimizer::VertexIDMap ref_vertices;
    SparseOptimizer::VertexIDMap curr_vertices;

    VirtualMatcher vm;

    // Load the vertices in the graphs
    for(SparseOptimizer::VertexIDMap::iterator it = merged_vertices.begin(); it != merged_vertices.end(); it++)
    {
        VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
        info._parent = NULL;
        info._transform = v->estimate().toIsometry();
        vm._currentInfo.insert(make_pair(v, info));

        if(v->id() < samples)
        {
            ref_vertices.insert(make_pair(v->id(), v));
        }
        else
        {
            curr_vertices.insert(make_pair(v->id(), v));
        }
        VertexSE2* copy = new VertexSE2;
        copy->setId(v->id());
        copy->setEstimate(v->estimate());

        output.addVertex(copy);
    }

    SparseOptimizer::EdgeSet edges = merged.edges();
    SparseOptimizer::EdgeSet ref_edges;
    SparseOptimizer::EdgeSet curr_edges;
    SparseOptimizer::EdgeSet closures;
    cout << "Adding edges" << endl;
    for(SparseOptimizer::EdgeSet::iterator it = edges.begin(); it != edges.end(); it++)
    {
        EdgeSE2* edge = dynamic_cast<EdgeSE2*>(*it);
        int from_id = edge->vertices()[0]->id();
        int to_id = edge->vertices()[1]->id();
        if(from_id < samples)
        {
            ref_edges.insert(edge);
            output.addEdge(edge);
        }
        if(from_id >= samples && to_id >= samples)
        {
            curr_edges.insert(edge);
            output.addEdge(edge);
        }
        if(from_id >= samples && to_id < samples)
        {
            closures.insert(edge);
        }
    }

    cout << "Ref_vert size: " << ref_vertices.size() << endl;
    cout << "Curr_vert size: " << curr_vertices.size() << endl;
    VertexSE2* first = dynamic_cast<VertexSE2*>(curr_vertices.find(samples)->second);
    vm.match(&ref_vertices, &curr_vertices, first, 2.0);

    EdgeSet matches = vm.results();
    for(EdgeSet::iterator it = matches.begin(); it != matches.end(); it++)
    {
        EdgeSE2* e = *it;
        output.addEdge(e);
    }

    ostringstream result;
    result << "result.g2o";
    output.save(result.str().c_str());

    cout << "Finished" << endl;

    Factory::destroy();
    OptimizationAlgorithmFactory::destroy();
    HyperGraphActionLibrary::destroy();

    return 0;
}
