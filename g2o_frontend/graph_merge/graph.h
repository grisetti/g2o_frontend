#ifndef GRAPH_H
#define GRAPH_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>


struct Graph;
struct Node;
struct Edge;


typedef std::set<Edge*> EdgeSet;
typedef std::set<Node*> NodeSet;
typedef std::map<Node*, EdgeSet> NodeEdgeMap;
typedef std::map<int, Edge*> EdgeMap;
typedef std::map<int, Node*> NodeMap;



struct GraphElement
{
    GraphElement(int id = -1, Graph* graph = 0);

    int _id;
    Graph* _graph;
};


struct Node : public GraphElement
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Node(int id = -1, Graph* graph = 0);

    const EdgeSet& edges();

    Eigen::Isometry2d _pose;
};


struct Edge : public GraphElement
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Edge(int id = -1, Graph* graph = 0);

    Node* _from, * _to;
    Eigen::Isometry2d _transform;
};


struct Graph
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Graph();

    Node* node(int id);
    bool addNode(Node* n);
    Edge* edge(int id);
    bool addEdge(Edge* e);

    const EdgeSet& edgesOf(Node* n);
    void initGraph(std::istream& is);

    NodeEdgeMap _nodeEdges;
    EdgeMap _edges;
    NodeMap _nodes;
};
#endif //GRAPH_H
