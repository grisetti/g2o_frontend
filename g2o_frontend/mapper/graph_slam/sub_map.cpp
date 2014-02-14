#include "sub_map.h"


using namespace g2o;
using namespace std;



SubMap::SubMap()
{
    _currentVertices = new HyperGraph::VertexSet;
    _currentDistance = 0;
    _overThreshold = false;
    _distanceThreshold = 0;
}


SubMap::SubMap(float distanceThreshold_)
{
    _currentVertices = new HyperGraph::VertexSet;
    _currentDistance = 0;
    _overThreshold = false;
    _distanceThreshold = distanceThreshold_;
}


bool SubMap::addToSubMap(HyperGraph::Vertex* v_)
{
     if(!_overThreshold)
    {
        if(_currentDistance <= _distanceThreshold)
        {
            _currentVertices->insert(v_);
            cout << "current vertex added to the local map" << endl;
            /*UPDATE CURRENT DISTANCE*/
        }
    }
    else
    {
        _currentVertices->erase(_currentVertices->begin());
        _currentVertices->insert(v_);
    }
    return true;
}
