#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "../matcher/matching/include.h"
#include "graph_cost_functions.h"
#include "sub_map.h"



class GraphSLAM
{
public:
    GraphSLAM(std::string filename);
    GraphSLAM(g2o::SparseOptimizer* graph_);

    void initialize(float distance_);
//    void findNearestVertices(g2o::OptimizableGraph::Vertex* v_, double threshold_);

    void localMapMatching();
//    void localMatching();
//    void loopClosureMatching();

//    void addLocalConstraint(int ref_id, int curr_id, Eigen::Isometry3d estimate);
//    void addLoopClosuresConstraints();


    inline g2o::SparseOptimizer* graph() {return _graph; }
    inline const g2o::SparseOptimizer* graph() const {return _graph; }
    inline SubMap* localMap() {return _innerMap; }
    inline const SubMap* localMap() const {return _innerMap; }

    inline SubMap* innerMap() { return _innerMap; }
    inline SubMap* outerMap() { return _outerMap; }

    bool saveGraph(const char* filename);

protected:
    g2o::SparseOptimizer* _graph;

    SubMap* _innerMap;
    SubMap* _outerMap;

//    Matcher _matcher;
};
#endif // GRAPHSLAM_H
