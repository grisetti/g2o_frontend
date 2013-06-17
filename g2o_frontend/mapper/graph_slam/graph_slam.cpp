#include "graph_slam.h"


using namespace g2o;
using namespace std;


typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;



GraphSLAM::GraphSLAM(string filename)
{
    _graph = new SparseOptimizer;
    _innerMap = new SubMap;
    _outerMap = new SubMap;

    SlamLinearSolver* linearSolver = new SlamLinearSolver;
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);

    _graph->setAlgorithm(solverGauss);
    _graph->load(filename.c_str());

    cout << "The graph contains: " << _graph->vertices().size() << endl;
}


GraphSLAM::GraphSLAM(g2o::SparseOptimizer* graph_)
{
    _graph = graph_;
    _innerMap = 0;
    _outerMap = 0;

    SlamLinearSolver* linearSolver = new SlamLinearSolver;
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);

    _graph->setAlgorithm(solverGauss);
}


void GraphSLAM::initialize(float distance_)
{
    _innerMap = new SubMap(distance_);
    _outerMap = new SubMap(distance_);
}


void GraphSLAM::localMapMatching()
{
    for(HyperGraph::VertexSet::const_iterator it = _innerMap->currentVertices()->begin(); it != _innerMap->currentVertices()->end(); ++it)
    {

    }
}

//void GraphSLAM::findNearestVertices(SparseOptimizer::Vertex* currentVertex_, double threshold_)
//{
//    HyperDijkstra hd(_graph);
//    EuclideanCostFunction ecf;
//    hd.shortestPaths(currentVertex_, &ecf, threshold_);

//    _matcher.setReferenceGauge(currentVertex_);
//    _matcher.addToCurrent(hd.visited());
//}


//void GraphSLAM::localMatching()
//{
//    HyperGraph::VertexSet _currentSet = _matcher.currentVerteces();
//    HyperGraph::Vertex* _referenceVertex = _matcher.referenceGauge();
//    HyperGraph::Vertex* _currentVertex = new HyperGraph::Vertex;
//    if(_currentSet.size() == 0)
//    {
//        cout << "Set for local matching is empty" << endl;
//        return;
//    }

//    for(HyperGraph::VertexSet::const_iterator it = _currentSet.begin(); it != _currentSet.end(); ++it)
//    {
//        _currentVertex = *it;
//        _matcher.match(_referenceVertex, _currentVertex);
//    }
//}


//void GraphSLAM::loopClosureMatching()
//{
//}

//void GraphSLAM::addLocalConstraint(int ref_id, int curr_id, Eigen::Isometry3d estimate)
//{

//}


//void GraphSLAM::addLoopClosuresConstraints()
//{

//}


bool GraphSLAM::saveGraph(const char *filename)
{
    return _graph->save(filename);
}
