#include "graph_slam.h"


using namespace g2o;
using namespace std;



GraphSLAM::GraphSLAM(OptimizableGraph* g_, Matcher m_)
{
    _graph = g_;
    _matcher = m_;
}


void GraphSLAM::findNearestVertices(OptimizableGraph::Vertex* currentVertex_, double threshold_)
{
    HyperDijkstra hd(_graph);
    EuclideanCostFunction ecf;
    hd.shortestPaths(currentVertex_, &ecf, threshold_);

    _matcher.setReferenceGauge(currentVertex_);
    _matcher.addToCurrent(hd.visited());
}


void GraphSLAM::localMatching()
{
    HyperGraph::VertexSet _currentSet = _matcher.currentVerteces();
    HyperGraph::Vertex* _referenceVertex = _matcher.referenceGauge();
    HyperGraph::Vertex* _currentVertex = new HyperGraph::Vertex;
    if(_currentSet.size() == 0)
    {
        cout << "Set for local matching is empty" << endl;
        return;
    }

    for(HyperGraph::VertexSet::const_iterator it = _currentSet.begin(); it != _currentSet.end(); ++it)
    {
        _currentVertex = *it;
        _matcher.match(_referenceVertex, _currentVertex);
    }
}


void GraphSLAM::loopClosureMatching()
{
}

void GraphSLAM::addLocalConstraint(int ref_id, int curr_id, Eigen::Isometry3d estimate)
{

}


void GraphSLAM::addLoopClosuresConstraints()
{

}
