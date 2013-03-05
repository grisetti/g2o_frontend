#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "../matcher/matching/include.h"
#include "graph_cost_functions.h"


class GraphSLAM
{
public:
    GraphSLAM(g2o::OptimizableGraph* graph_, Matcher m_);

    void findNearestVertices(g2o::OptimizableGraph::Vertex* v_, double threshold_);
    void localMatching();
    void addLocalConstraints();
    void addLoopClosuresConstraints();

protected:
    Matcher _matcher;
    g2o::OptimizableGraph* _graph;

};

#endif // GRAPHSLAM_H
