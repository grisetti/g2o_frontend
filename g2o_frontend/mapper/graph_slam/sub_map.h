#ifndef SUBMAP_H
#define SUBMAP_H

#include <iostream>
#include "g2o/core/hyper_graph.h"


class SubMap
{
public:
    SubMap();
    SubMap(float distanceThreshold_);

    bool addToSubMap(g2o::HyperGraph::Vertex* v_);

    inline void setDistanceThreshold(float distanceThreshold_) { _distanceThreshold = distanceThreshold_; }
    inline float distanceThreshold() { return _distanceThreshold; }
    inline g2o::HyperGraph::VertexSet* currentVertices() {return _currentVertices; }
    inline const g2o::HyperGraph::VertexSet* currentVertices() const {return _currentVertices; }

protected:
    g2o::HyperGraph::VertexSet* _currentVertices;

    float _distanceThreshold;
    float _currentDistance;
    bool _overThreshold;
};


#endif // SUBMAP_H
