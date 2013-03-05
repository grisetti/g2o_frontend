#ifndef GRAPHCOSTFUNCTIONS_H
#define GRAPHCOSTFUNCTIONS_H

#include "g2o/core/hyper_dijkstra.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"



class EuclideanCostFunction : public g2o::HyperDijkstra::CostFunction
{

public:
    EuclideanCostFunction();

    virtual double operator()(g2o::HyperGraph::Edge* e_, g2o::HyperGraph::Vertex* from_, g2o::HyperGraph::Vertex* to_)
    {
        g2o::EdgeSE2* _e = dynamic_cast<g2o::EdgeSE2*>(e_);

        if(!_e)
        {
            std::cout << "Dynamic cast failed" << std::endl;
            return std::numeric_limits<double>::max();
        }

        g2o::VertexSE2* _from = dynamic_cast<g2o::VertexSE2*>(from_);
        g2o::VertexSE2* _to = dynamic_cast<g2o::VertexSE2*>(to_);

        if(_from && _to)
        {
            Eigen::Vector2d distance = _from->estimate().translation() - _to->estimate().translation();
            return distance.norm();
        }

        else
        {
            std::cout << "Could not calculate correct edge length" << std::endl;
            return std::numeric_limits<double>::max();
        }
    }
};

#endif // GRAPHCOSTFUNCTIONS_H
