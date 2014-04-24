#include "graph.h"
#include "utility.h"

#include <stdexcept>

using namespace std;
using namespace Eigen;


GraphElement::GraphElement(int id, Graph* graph)
{
    this->_id = id;
    this->_graph = graph;
}


Node::Node(int id, Graph* graph) : GraphElement(id, graph)
{
    _pose.setIdentity();
}


bool Node::addLaser(RobotLaser* l)
{
    RobotLaser* l1 = this->_laser;
    if(l1)
    {
        return false;
    }
    this->_laser = l1;
    return true;
}


const EdgeSet& Node::edges()
{
    return _graph->edgesOf(this);
}


RobotLaser::RobotLaser(Node* node)
{
    this->_node = node;
}


RobotLaser::Vector2fVector RobotLaser::floatCartesian() const
{
    Vector2fVector points;
    points.reserve(_ranges.size());
    float angularStep = _fov / _ranges.size();
    float alpha=_firstBeamAngle;
    for(size_t i = 0; i < _ranges.size(); ++i)
    {
        const float& r = _ranges[i];
        if(r < _maxRange)
        {
            points.push_back(Eigen::Vector2f(cos(alpha) * r, sin(alpha) * r));
        }
        alpha += angularStep;
    }
    return points;
}


Edge::Edge(int id, Graph* graph) : GraphElement(id, graph)
{
    _from = 0;
    _to = 0;

    _transform.setIdentity();
}


Graph::Graph() {}


Node* Graph::node(int id)
{
    NodeMap::iterator it = _nodes.find(id);
    if(it != _nodes.end())
    {
        return it->second;
    }
    return 0;
}


bool Graph::addNode(Node* n)
{
    Node* n2 = node(n->_id);
    if(n2)
    {
        return false;
    }
    _nodes.insert(make_pair(n->_id, n));
    _nodeEdges.insert(make_pair(n, EdgeSet()));
    return true;
}


Edge* Graph::edge(int id)
{
    EdgeMap::iterator it = _edges.find(id);
    if(it != _edges.end())
    {
        return it->second;
    }
    return 0;
}


bool Graph::addEdge(Edge* e)
{
    Edge* e2 = edge(e->_id);
    if(e2)
    {
        return false;
    }
    _edges.insert(make_pair(e->_id, e));
    _nodeEdges[e->_from].insert(e);
    _nodeEdges[e->_to].insert(e);
    return true;
}


const EdgeSet& Graph::edgesOf(Node* n)
{
    NodeEdgeMap::iterator it = _nodeEdges.find(n);
    if(it == _nodeEdges.end())
    {
        throw std::runtime_error("unable to find an entry in the node edge map");
    }
    return it->second;
}


void Graph::initGraph(istream& is)
{
    string tag;
    const int maxDim = 32000;
    int eid = 0;
    Node* previous = 0;
    while(is)
    {
        char line[maxDim];
        is.getline(line, maxDim);
        //  cerr << "line: " << line << endl;
        istringstream ls(line);
        ls >> tag;
        if(tag == "VERTEX_SE2")
        {
            int id;
            Vector3d v;
            ls >> id >> v.x() >> v.y() >> v.z();
            Node* n = new Node(id, this);
            n->_pose = utility::v2t(v);
            this->addNode(n);
            previous = n;
        }
        else if(tag == "ROBOTLASER1")
        {
//            cout << "LASER" << endl;
//            RobotLaser* rl = new RobotLaser();
//            float res, remissionMode;
//            is >> rl->_paramIndex >> rl->_firstBeamAngle >> rl->_fov >> res >> rl->_maxRange >> rl->_accuracy >> remissionMode;

//            int beams;
//            is >> beams;
//            rl->_ranges.resize(beams);
//            for(int i = 0; i < beams; i++)
//            {
//                is >> rl->_ranges[i];
//            }

//            is >> beams;
//            rl->_intensities.resize(beams);
//            for(int i = 0; i < beams; i++)
//            {
//                is >> rl->_intensities[i];
//            }

//            // special robot laser stuff
//            double x,y,theta;
//            //odom pose of the robot: no need
//            is >> x >> y >> theta;
//            //laser pose wrt to the world: no need
//            is >> x >> y >> theta;
//            is >> rl->_laserTv >>  rl->_laserRv >>  rl->_forwardSafetyDist >> rl->_sideSaftyDist >> rl->_turnAxis;

//            // timestamp + hostname
//            string hostname;
//            double ts;
//            is >> ts;
//            is >> hostname;
//            rl->_timestamp = ts;
//            is >> ts;

//            previous->addLaser(rl);
        }
        else if(tag == "EDGE_SE2")
        {
            int id1, id2;
            Vector3d v;
            ls >> id1 >> id2 >> v.x() >> v.y() >> v.z();
            Edge* e = new Edge(eid++, this);
            e->_from = this->node(id1);
            e->_to = this->node(id2);
            e->_transform = utility::v2t(v);
            this->addEdge(e);
        }
        else
        {

        }
    }
}
