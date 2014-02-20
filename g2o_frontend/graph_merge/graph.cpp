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


const EdgeSet& Node::edges()
{
    return _graph->edgesOf(this);
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
//            cerr << "Tag: " << tag << endl;
        }
    }
}
