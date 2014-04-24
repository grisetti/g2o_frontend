#ifndef GRAPH_H
#define GRAPH_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <set>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>


struct Graph;
struct Node;
struct Edge;
struct RobotLaser;


typedef std::set<Edge*> EdgeSet;
typedef std::set<Node*> NodeSet;
typedef std::map<Node*, EdgeSet> NodeEdgeMap;
typedef std::map<int, Edge*> EdgeMap;
typedef std::map<int, Node*> NodeMap;



struct GraphElement
{
    GraphElement(int id = -1, Graph* graph = 0);
    ~GraphElement();

    int _id;
    Graph* _graph;
};


struct Node : public GraphElement
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Node(int id = -1, Graph* graph = 0);

    bool addLaser(RobotLaser* l);
    const EdgeSet& edges();

    Eigen::Isometry2d _pose;
    RobotLaser* _laser;
};


struct RobotLaser
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

    RobotLaser(Node* node = 0);

    Vector2fVector floatCartesian() const;

    const std::vector<float>& ranges() const {return _ranges;}
    std::vector<float>& ranges() {return _ranges;}
    const std::vector<float>& intensities() const {return _intensities;}
    std::vector<float>& intensities() {return _intensities;}

    float minRange() const { return _minRange; }
    void  setMinRange(float r) { _minRange = r; }

    float maxRange() const { return _maxRange; }
    void  setMaxRange(float r)  { _maxRange = r; }

    float fov() const { return _fov; }
    void setFov(float f) { _fov = f; }

    float firstBeamAngle() const { return _firstBeamAngle; }
    void setFirstBeamAngle(float fba)  { _firstBeamAngle = fba; }

    Node* _node;

    //! velocities and safety distances of the robot
    double _laserTv, _laserRv, _forwardSafetyDist, _sideSaftyDist, _turnAxis;
    std::vector<float> _ranges;
    std::vector<float> _intensities;
    float _firstBeamAngle;
    float _fov;
    float _minRange;
    float _maxRange;
    float _accuracy;
    double _timestamp;
    int _paramIndex;
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

    inline const EdgeMap& edgeMap() { return this->_edges; }
    inline const NodeEdgeMap& nodeEdgeMap() { return this->_nodeEdges; }
    inline const NodeMap& nodeMap() { return this->_nodes; }


    EdgeMap _edges;
    NodeEdgeMap _nodeEdges;
    NodeMap _nodes;
};
#endif //GRAPH_H
