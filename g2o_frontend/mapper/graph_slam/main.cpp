#include "graph_slam.h"

#include "g2o/stuff/command_args.h"


using namespace g2o;
using namespace std;


volatile bool hasToStop;


int main(int argc, char **argv)
{
    hasToStop = false;
    string filename;

    CommandArgs arg;
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);

    if(filename.size() == 0)
    {
        cerr << "No input data specified" << endl;
        return 0;
    }


//    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
//    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//    SlamLinearSolver* linearSolver = new SlamLinearSolver();
//    linearSolver->setBlockOrdering(false);
//    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
//    OptimizationAlgorithmGaussNewton* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
//    SparseOptimizer* graph = new SparseOptimizer();
//    graph->setAlgorithm(solverGauss);
    OptimizableGraph* graph = new OptimizableGraph;
    graph->load(filename.c_str());


    // sort the vertices based on the id
    vector<int> vertexIds(graph->vertices().size());
    cout << "vertices: " << graph->vertices().size() << endl;
    int k=0;
    for (OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it)
    {
        vertexIds[k++] = (it->first);
    }

    sort(vertexIds.begin(), vertexIds.end());

    cout << "vertices: " << vertexIds.size() << endl;

//    GraphSLAM gslam(filename);
//    gslam.initialize(10000);
//    cout << gslam.localMap()->distanceThreshold() << endl;

//    HyperGraph::Vertex* v = new HyperGraph::Vertex;
//    gslam.localMap()->addToLocalMap(v);
//    cout << "size: " << gslam.localMap()->currentVertices()->size() << endl;

//    HyperGraph::Vertex* v1 = new HyperGraph::Vertex;
//    gslam.localMap()->addToLocalMap(v1);
//    cout << "size: " << gslam.localMap()->currentVertices()->size() << endl;


//    cout << "boh: " << gslam.graph()->vertices().size() << endl;

//    // sort the vertices based on the id
//    vector<int> vertexIds(gslam.graph()->vertices().size());
//    cout << "The Graph contains " << vertexIds.size() << " nodes" << endl;
//    int k=0;
//    for (OptimizableGraph::VertexIDMap::iterator it = gslam.graph()->vertices().begin(); it != gslam.graph()->vertices().end(); ++it)
//    {
//        vertexIds[k++] = (it->first);
//    }

//    sort(vertexIds.begin(), vertexIds.end());
//    for(size_t i = 0; i < vertexIds.size() && ! hasToStop; ++i)
//    {

//    }

    exit(0);
}
