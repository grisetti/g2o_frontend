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


using namespace g2o;
using namespace std;


double computeScore(OptimizableGraph* g1, OptimizableGraph* g2)
{
    SparseOptimizer::VertexIDMap g1ver = g1->vertices();
    SparseOptimizer::VertexIDMap g2ver = g2->vertices();

    SparseOptimizer::VertexIDMap::const_iterator g1t, g2t;
    double sum = -1;
    for(g1t = g1ver.begin(), g2t = g2ver.begin(); g1t != g1ver.end() && g2t != g2ver.end(); g1t++, g2t++)
    {
        VertexSE2* gv = dynamic_cast<VertexSE2*>(g1t->second);
        VertexSE2* cv = dynamic_cast<VertexSE2*>(g2t->second);

        SE2 gvt = gv->estimate();
        SE2 cvt = cv->estimate();
        SE2 dt = gvt.inverse() * cvt;

        double score = dt.toVector().squaredNorm();
        sum += score;
    }
    cout << sum << endl;
    return sum;
}



int main(int argc, char** argv)
{
    int samples = 1500;
    char* gt = 0;
    char* filename = 0;
    int maxIter = -1;

    if(argc == 3)
    {
        gt = argv[1];
        filename = argv[2];
    }
    else if(argc == 4)
    {
        gt = argv[1];
        maxIter = atoi(argv[argc-1]);
        filename = argv[2];
    }
    else
    {
        cout << "Wrong number of input parameters" << endl;
        return -1;
    }

    cout << "Gt: " << gt << endl;
    cout << "File: " << filename << endl;

    ofstream ofs("errors.txt");

    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    int iter = 0;
    while(iter < samples + 1)
    {
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

        SparseOptimizer ground;
        SlamLinearSolver* linearSolver2 = new SlamLinearSolver();
        linearSolver2->setBlockOrdering(false);
        SlamBlockSolver* blockSolver2 = new SlamBlockSolver(linearSolver2);
        OptimizationAlgorithmGaussNewton* solver2 = new OptimizationAlgorithmGaussNewton(blockSolver2);
        ground.setAlgorithm(solver2);
        ground.load(gt);

        SparseOptimizer::VertexIDMap merged_vertices = merged.vertices();

        SparseOptimizer::VertexIDMap ref_vertices;
        SparseOptimizer::VertexIDMap curr_vertices;

        GraphMatcher vm;
        IdealNodeMatcher* im = new IdealNodeMatcher;

        int offset = samples + 100000;

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
        vm.match(&ref_vertices, first, 25.0, iter);

        EdgeSet matches = vm.results();
        for(EdgeSet::iterator it = matches.begin(); it != matches.end(); it++)
        {
            EdgeSE2* e = *it;
            output.addEdge(e);
        }


        VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(output.vertex(0));
        firstRobotPose->setFixed(true);
        output.setVerbose(true);

//        output.initializeOptimization();
//        output.optimize(10);

        ostringstream result;
        result << iter << "_iterations.g2o";
        output.save(result.str().c_str());

        SparseOptimizer output1;
        SlamLinearSolver* linearSolver3 = new SlamLinearSolver();
        linearSolver3->setBlockOrdering(false);
        SlamBlockSolver* blockSolver3 = new SlamBlockSolver(linearSolver3);
        OptimizationAlgorithmGaussNewton* solver3 = new OptimizationAlgorithmGaussNewton(blockSolver3);
        output1.setAlgorithm(solver3);
        output1.load(result.str().c_str());

        cout << "reloading " << result.str().c_str() << endl;

        double res = computeScore(&ground, &output1);
        ofs << "\t" << iter << "\t\t" << res << endl;

        iter += 10;
    }
    cout << "Finished" << endl;
    return 0;
}



//int main(int argc, char** argv)
//{
//    if(argc < 3 || argc > 5)
//    {
//        cout << "Wrong arguments number" << endl;
//    }

//    char* gt = argv[1];
//    char* in = argv[2];

//    cout << "Ground-truth file: " << gt << endl;
//    cout << "Reconstructed file: " << in << endl;

//    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
//    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

//    // allocating the optimizer
//    SparseOptimizer ground;
//    SlamLinearSolver* linearSolver = new SlamLinearSolver();
//    linearSolver->setBlockOrdering(false);
//    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
//    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
//    ground.setAlgorithm(solver);
//    ground.load(gt);

//    SparseOptimizer reconst;
//    SlamLinearSolver* linearSolver1 = new SlamLinearSolver();
//    linearSolver1->setBlockOrdering(false);
//    SlamBlockSolver* blockSolver1 = new SlamBlockSolver(linearSolver1);
//    OptimizationAlgorithmGaussNewton* solver1 = new OptimizationAlgorithmGaussNewton(blockSolver1);
//    reconst.setAlgorithm(solver1);
//    reconst.load(in);

//    SparseOptimizer::VertexIDMap gtvert = ground.vertices();
//    SparseOptimizer::VertexIDMap revert = reconst.vertices();

//    SparseOptimizer::VertexIDMap::const_iterator it;
//    SparseOptimizer::VertexIDMap::const_iterator rit;

//    double sum = -1;
//    for(it = gtvert.begin(), rit = revert.begin(); it != gtvert.end() && rit != revert.end(); it++, rit++)
//    {
//        VertexSE2* vgt = dynamic_cast<VertexSE2*>(it->second);
//        VertexSE2* vre = dynamic_cast<VertexSE2*>(rit->second);

//        SE2 vt = vgt->estimate();
//        SE2 rt = vre->estimate();
//        SE2 tsf = vt.inverse() * rt;
//        double score = tsf.toVector().squaredNorm();
//        sum += score;

////        cout << "GT:" << vgt->id() << ", T: " << vt.toVector().x() << " " << vt.toVector().y() << " " << vt.toVector().z() << endl;
////        cout << "GT:" << vre->id() << ", T: " << rt.toVector().x() << " " << rt.toVector().y() << " " << rt.toVector().z() << endl;
////        cout << "Score: " << score << endl;
//    }
//    cout << sum << endl;

//    return 0;
//}
