#include <signal.h>

#include "map_gui.h"

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

#include <qapplication.h>
#include <qobject.h>


using namespace std;
using namespace g2o;



//to be deleted?
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
//    hasToStop = false;
//    string filename1;
//    string filename2;

//    CommandArgs arg;
//    arg.paramLeftOver("graph-input", filename1, "", "graph file which will be processed", true);
//    arg.paramLeftOver("graph-input", filename2, "", "graph file which will be processed", true);
//    arg.parseArgs(argc, argv);

//    // graph construction
//    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
//    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//    SlamLinearSolver* linearSolver = new SlamLinearSolver();
//    linearSolver->setBlockOrdering(false);
//    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
//    OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
//    SparseOptimizer * graph1 = new SparseOptimizer();
//    graph1->setAlgorithm(solverGauss);
//    graph1->load(filename1.c_str());


//    // sort the vertices based on the id
//    vector<int> vertexIds(graph1->vertices().size());
//    int k = 0;

//    for(OptimizableGraph::VertexIDMap::iterator it = graph1->vertices().begin(); it != graph1->vertices().end(); ++it)
//    {
//        vertexIds[k++] = (it->first);
//    }
//    sort(vertexIds.begin(), vertexIds.end());

//    Vertices drawableVertices;
//    Edges drawableEdges;

//    for(size_t i = 0; i < vertexIds.size() && ! hasToStop; ++i)
//    {
//        HyperGraph::Vertex* v = graph1->vertex(vertexIds[i]);
//        if(!v)
//        {
//            continue;
//        }

//        HyperGraph::EdgeSet edgeSet = v->edges();
//        for(HyperGraph::EdgeSet::const_iterator it = edgeSet.begin(); it != edgeSet.end(); ++it)
//        {
//            HyperGraph::Edge* e = *it;
//            drawableEdges.push_back(e);
//        }
//        drawableVertices.push_back(v);
//    }
//    cout << "First graph read" << endl;


//    SparseOptimizer* graph2 = new SparseOptimizer;
//    graph2->setAlgorithm(solverGauss);
//    graph2->load(filename2.c_str());

//    vector<int> verticesIds(graph2->vertices().size());
//    int k2 = 0;


//    for(OptimizableGraph::VertexIDMap::iterator it = graph2->vertices().begin(); it != graph2->vertices().end(); ++it)
//    {
//        verticesIds[k2++] = (it->first);
//    }
//    sort(verticesIds.begin(), verticesIds.end());

//    Vertices drawableVertices2;
//    Edges drawableEdges2;

//    for(size_t i = 0; i < verticesIds.size() && ! hasToStop; ++i)
//    {
//        HyperGraph::Vertex* v = graph2->vertex(verticesIds[i]);
//        if(!v)
//        {
//            continue;
//        }

//        HyperGraph::EdgeSet edgeSet = v->edges();
//        for(HyperGraph::EdgeSet::const_iterator it = edgeSet.begin(); it != edgeSet.end(); ++it)
//        {
//            HyperGraph::Edge* e = *it;
//            drawableEdges2.push_back(e);
//        }
//        drawableVertices2.push_back(v);
//    }
//    cout << "Second graph read" << endl;

    QApplication app(argc, argv);
    MapGUI* dialog = new MapGUI();
    dialog->show();

    return app.exec();
}
