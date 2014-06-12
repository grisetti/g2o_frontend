#include <iostream>
#include <cmath>
#include <vector>

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "graph_matcher.h"
#include "graph_simulator.h"

#define EXP_ITER 10

using namespace g2o;
using namespace std;


//static double computeScore(OptimizableGraph* g1, OptimizableGraph* g2)
//{
//    SparseOptimizer::VertexIDMap g1ver = g1->vertices();
//    SparseOptimizer::VertexIDMap g2ver = g2->vertices();

//    SparseOptimizer::VertexIDMap::const_iterator g1t, g2t;
//    double sum = -1;
//    for(g1t = g1ver.begin(), g2t = g2ver.begin(); g1t != g1ver.end() && g2t != g2ver.end(); g1t++, g2t++)
//    {
//        VertexSE2* gv = dynamic_cast<VertexSE2*>(g1t->second);
//        VertexSE2* cv = dynamic_cast<VertexSE2*>(g2t->second);

//        SE2 gvt = gv->estimate();
//        SE2 cvt = cv->estimate();
//        SE2 dt = gvt.inverse() * cvt;

//        double score = dt.toVector().squaredNorm();
//        sum += score;
//    }
//    cout << sum << endl;
//    return sum;
//}


static double computeScore(OptimizableGraph* g1, OptimizableGraph* g2)
{
    SparseOptimizer::VertexIDMap& g1ver = g1->vertices();

    SparseOptimizer::VertexIDMap::const_iterator g1t;
    double sum = 0;
    for(g1t = g1ver.begin(); g1t != g1ver.end() ; g1t++)
    {
        VertexSE2* gv = dynamic_cast<VertexSE2*>(g1t->second);
        VertexSE2* cv = dynamic_cast<VertexSE2*>(g2->vertex(gv->id()));

        SE2 gvt = gv->estimate();
        SE2 cvt = cv->estimate();
        SE2 dt = gvt.inverse() * cvt;

        double score = dt.toVector().squaredNorm();
        sum += score;
    }
    cout << sum << endl;
    return sum;
}


int main()
{
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    SparseOptimizer ground;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
    ground.setAlgorithm(solver);

    SparseOptimizer nomer;
    SlamLinearSolver* linearSolver1 = new SlamLinearSolver();
    linearSolver1->setBlockOrdering(false);
    SlamBlockSolver* blockSolver1 = new SlamBlockSolver(linearSolver1);
    OptimizationAlgorithmGaussNewton* solver1 = new OptimizationAlgorithmGaussNewton(blockSolver1);
    nomer.setAlgorithm(solver1);

    SparseOptimizer output;
    SlamLinearSolver* linearSolver2 = new SlamLinearSolver();
    linearSolver2->setBlockOrdering(false);
    SlamBlockSolver* blockSolver2 = new SlamBlockSolver(linearSolver2);
    OptimizationAlgorithmGaussNewton* solver2 = new OptimizationAlgorithmGaussNewton(blockSolver2);
    output.setAlgorithm(solver2);

    ofstream ofs("new-noise-results.dat");
    ofstream avfs("stats.dat");

    int i = 0;

    float nx = 0.05;
    float ny = 0.01;
    float nz = 2;

    // Noise on th
//    for(float nz = 2.; nz <= 3.; nz += 0.25)
//    {
//        // Noise on x
//        for(float nx = 0.05; nx <= 0.25; nx += 0.05)
//        {
//            // Noise on y
//            for(float ny = 0.01; ny <= 0.1; ny += 0.02)
//            {
                double sum_no = 0;
                double sum_mer = 0;
                vector<double> no_m;
                vector<double> to_m;
                for(i = 0; i < EXP_ITER; ++i)
                {
                    // Multi simulation merge
                    ostringstream msmcomm;
                    msmcomm << "multi_simulation_merge " << nx << " " << ny << " " << nz;
                    system(msmcomm.str().c_str());

                    ostringstream iter;
                    iter << "-" << i+1 << "-" << nx << "-" << ny << "-" << nz;

                    ostringstream clname;
                    clname << "closures" << iter.str().c_str() << ".g2o";

                    ostringstream clcomm;
                    clcomm << "mv closures.g2o " << clname.str().c_str();
                    system(clcomm.str().c_str());

                    ostringstream gtname;
                    gtname << "ground-truth" << iter.str().c_str() << ".g2o";

                    ostringstream gtcomm;
                    gtcomm << "mv merged_after.g2o " << gtname.str().c_str();
                    system(gtcomm.str().c_str());

                    ostringstream nmname;
                    nmname << "no-merge" << iter.str().c_str() << ".g2o";

                    ostringstream fmcomm;
                    fmcomm << "file_merge after-0.g2o after-1.g2o " << nmname.str().c_str();
                    system(fmcomm.str().c_str());

                    ostringstream tmname;
                    tmname << "to-merge" << iter.str().c_str() << ".g2o";

                    ostringstream catcomm;
                    catcomm << "cat " << nmname.str().c_str()
                            << " " << clname.str().c_str()
                            << " > " << tmname.str().c_str();
                    system(catcomm.str().c_str());

                    ostringstream mmcomm;
                    mmcomm << "match_merge " << gtname.str().c_str()
                           << " " << tmname.str().c_str();
                    system(mmcomm.str().c_str());

                    ostringstream otname;
                    otname << "result" << iter.str().c_str() << ".g2o";

                    ostringstream otcomm;
                    otcomm << "mv simulated_result.g2o " << otname.str().c_str();
                    system(otcomm.str().c_str());

                    ground.clear();
                    nomer.clear();
                    output.clear();
                    ground.load(gtname.str().c_str());
                    nomer.load(nmname.str().c_str());
                    output.load(otname.str().c_str());

                    double resnomer = computeScore(&nomer, &ground);
                    double restomer = computeScore(&output, &ground);

                    sum_no += resnomer;
                    sum_mer += restomer;

                    ofs << nx << "\t\t" << ny << "\t\t" << "\t\t" << nz << "\t\t"
                        << resnomer << "\t\t" << restomer << endl;

                    no_m.push_back(resnomer);
                    to_m.push_back(restomer);
                }
                double no_avg = sum_no/EXP_ITER;
                double to_avg = sum_mer/EXP_ITER;

                double no_var = 0;
                double to_var = 0;

                for(int k = 0; k < EXP_ITER; k++)
                {
                    no_var += (no_m[k] - no_avg)*(no_m[k] - no_avg);
                    to_var += (to_m[k] - to_avg)*(to_m[k] - to_avg);
                }
                no_var = no_var/EXP_ITER;
                to_var = to_var/EXP_ITER;

                avfs << nx << "\t\t" << ny << "\t\t" << "\t\t" << nz << "\t\t"
                     << no_avg << "\t\t" << sqrt(no_var) << "\t\t"
                     << to_avg << "\t\t" << sqrt(to_var) << endl;
//            }
//        }
//    }
    ofs.close();
    avfs.close();

    cout << "Finished" << endl;
    return 0;
}
