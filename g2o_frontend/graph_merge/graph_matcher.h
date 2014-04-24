#ifndef GRAPH_MATCHER_H
#define GRAPH_MATCHER_H

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include <set>
#include <vector>

class Edge;
class Graph;
class Node;

typedef std::vector<Graph*> Trajectories;
typedef std::set<Edge*> Matches;

struct GraphMatcher
{
public:
    GraphMatcher();
    GraphMatcher(Graph* g1, Graph* g2);
    ~GraphMatcher();

    void createGraph(const char* in1, const char* in2, const char* out);
    void recursiveMatch(Node *n1, Node *n2);
    void circularMatch(Node *n1, Node *n2);

    bool pushGraph(Graph* g);
    void saveGraph(g2o::OptimizableGraph& output);

    Matches& matches() { return _matches; }
    const Trajectories& graphs() { return _graphs; }

protected:
    Graph* _g1, *_g2;
    Trajectories _graphs;

    Matches _matches;
    std::set<Node*> _visited1, _visited2;

    double visitDistance;
    double _nodeDistance, _nodeRotation;
};
#endif // GRAPH_MATCHER_H
