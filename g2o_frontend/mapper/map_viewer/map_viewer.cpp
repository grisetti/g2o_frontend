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
    hasToStop = false;
    string filename;
    CommandArgs arg;
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);

    // graph construction
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
    SparseOptimizer * graph = new SparseOptimizer();
    graph->setAlgorithm(solverGauss);
    graph->load(filename.c_str());

    // sort the vertices based on the id
    vector<int> vertexIds(graph->vertices().size());
    int k = 0;

    for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it)
    {
      vertexIds[k++] = (it->first);
    }
    sort(vertexIds.begin(), vertexIds.end());

    Vertices drawableVertices;
    Edges drawableEdges;

    for(size_t i = 0; i < vertexIds.size() && ! hasToStop; ++i)
    {
        HyperGraph::Vertex* v = graph->vertex(vertexIds[i]);
        if(!v)
        {
            continue;
        }

        HyperGraph::EdgeSet edgeSet = v->edges();
        for(HyperGraph::EdgeSet::const_iterator it = edgeSet.begin(); it != edgeSet.end(); ++it)
        {
            HyperGraph::Edge* e = *it;
            drawableEdges.push_back(e);
        }
        drawableVertices.push_back(v);
    }
    cout << "End of file!" << endl;

    QApplication app(argc, argv);
    MapGUI* dialog = new MapGUI(&drawableVertices, &drawableEdges);
    dialog->viewer->setDataPointer(drawableVertices, drawableEdges);
    dialog->show();

    return app.exec();
}
