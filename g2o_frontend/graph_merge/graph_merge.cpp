#include <QApplication>

#include "drawable.h"
#include "graph.h"
#include "graph_matcher.h"
#include "graph_gui.h"
#include "viewer.h"
#include "utility.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"


#define MARKUSED(x) x=x


using namespace std;
using namespace Eigen;
using namespace g2o;


void transformGraph(Graph* g, Isometry2d tsf)
{
    NodeMap nm = g->nodeMap();
    for(NodeMap::iterator it = nm.begin(); it != nm.end(); it++)
    {
        Node* n = it->second;
        n->_pose = tsf.inverse() * n->_pose;
    }
}


/** MAIN FOR REAL GRAPHS*/
int main(int argc, char** argv)
{
 /*   typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    SparseOptimizer optimizer;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
    optimizer.setAlgorithm(solver);


    GraphMatcher gm;
    int c = 1;
    while(c < argc - 1)
    {
        cerr << "Loading file " << argv[c] << endl;
        ifstream is(argv[c]);
        Graph* g = new Graph;
        g->initGraph(is);
        gm.pushGraph(g);
        c++;
    }

    QApplication* app = new QApplication(argc, argv);
    GraphGUI win;
    win.show();
    float k = 0.35;
    for(size_t i = 0; i < gm.graphs().size(); i++)
    {
        cout << endl << "Evaluating graph: " << i+1 << endl;
        Graph* g = gm.graphs()[i];
        GraphDrawableItem* gdi = new GraphDrawableItem(g, win.graphViewer());
        gdi->_g = k * (i + 1);
        gdi->_b = k * (i + 1);
        MARKUSED(gdi);
    }

    Graph* g0 = gm.graphs()[0];
    Graph* g1 = gm.graphs()[1];

    Isometry2d init = g0->node(0)->_pose.inverse() * g1->node(0)->_pose;
    transformGraph(g1, init);

    gm.circularMatch(g0->node(0), g1->node(0));

    set<Edge*> matches = gm.matches();
    if(matches.size() == 0)
    {
        cerr << "No matches found" << endl;
    }
    else
    {
        cout << "Discovered following matches:" << endl;
        for(set<Edge*>::iterator it = matches.begin(); it != matches.end(); it++)
        {
            Edge* e = *it;
            cout << "EDGE: " << e->_from->_id << " " << e->_to->_id << endl;
        }
        AssociationDrawableItem* adi = new AssociationDrawableItem(&matches, win.graphViewer());
        MARKUSED(adi);
    }

    gm.saveGraph(optimizer);
//    gm.createGraph(argv[1], argv[2], argv[3]);
    while(win.graphViewer()->isVisible())
    {
        app->processEvents();
        usleep(1000);
    }*/
}
