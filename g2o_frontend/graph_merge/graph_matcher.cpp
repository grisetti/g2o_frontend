#include "graph.h"
#include "graph_matcher.h"
#include "utility.h"

#include <deque>
#include <iostream>
#include <vector>


using namespace std;
using namespace Eigen;
using namespace g2o;



GraphMatcher::GraphMatcher()
{
    _g1 = 0;
    _g2 = 0;
}


GraphMatcher::GraphMatcher(Graph* g1, Graph* g2)
{
    _g1 = g1;
    _g2 = g2;

    _graphs.push_back(g1);
    _graphs.push_back(g2);
}


GraphMatcher::~GraphMatcher(){}


bool GraphMatcher::pushGraph(Graph* g)
{
    if(_g1 && _g2)
    {
        cout << "Graphs already initialized" << endl;
        return false;
    }
    if(!_g1 && _g2)
    {
        cout << "This should not happen" << endl;
        return false;
    }

    if(!_g1 && !_g2)
    {
        _g1 = g;
        this->_graphs.push_back(g);
        cout << "First graph initialized" << endl;
        return true;
    }

    if(_g1 && !_g2)
    {
        _g2 = g;
        this->_graphs.push_back(g);
        cout << "Second graph initialized" << endl;
        return true;
    }
}


void GraphMatcher::saveGraph(OptimizableGraph &output)
{
    int offset = 10000;

    Matrix3d information;
    information.fill(0.);
    information(0, 0) = 100;
    information(1, 1) = 100;
    information(2, 2) = 1000;

    for(size_t k = 0; k < _graphs.size(); ++k)
    {
        Graph* traj = _graphs[k];
        NodeMap nodes = traj->nodeMap();
        for(NodeMap::const_iterator it = nodes.begin(); it != nodes.end(); it++)
        {
            Node* n = it->second;
            int new_id = n->_id + k * offset;

            VertexSE2* v = new VertexSE2;
            v->setId(new_id);
            v->setEstimate(n->_pose);

            output.addVertex(v);
        }

        EdgeMap edges = traj->edgeMap();
        for(EdgeMap::iterator eit = edges.begin(); eit != edges.end(); eit++)
        {
            Edge* edge = eit->second;

            EdgeSE2* odometry = new EdgeSE2;
            int new_from_id = edge->_from->_id + k * offset;
            int new_to_id = edge->_to->_id + k * offset;
            odometry->vertices()[0] = output.vertex(new_from_id);
            odometry->vertices()[1] = output.vertex(new_to_id);
            odometry->setMeasurement(edge->_transform);
            odometry->setInformation(information);

            output.addEdge(odometry);
        }
    }

    for(Matches::iterator mit = this->_matches.begin(); mit != this->_matches.end(); mit++)
    {
        Edge* edge = *mit;

        EdgeSE2* closure = new EdgeSE2;
        int new_from_id = edge->_from->_id + offset;
        closure->vertices()[0] = output.vertex(new_from_id);
        closure->vertices()[1] = output.vertex(edge->_to->_id);
        closure->setMeasurement(edge->_transform);
        closure->setInformation(information);

        output.addEdge(closure);
    }

    ostringstream out;
    out << "closed_graph.g2o";
    output.save(out.str().c_str());
}


void GraphMatcher::createGraph(const char* in1, const char* in2, const char* out)
{
    int offset = 10000;

    vector<const char*> vec;

    vec.push_back(in1);
    vec.push_back(in2);

    ofstream os(out);
    for(short unsigned int i = 0; i < vec.size(); i++)
    {
        ifstream file(vec[i]);
        string tag;
        const int maxDim = 32000;
        int counter = 0;
        while(file)
        {
            char line[maxDim];
            file.getline(line, maxDim);
            istringstream ls(line);
            ls >> tag;
            if(tag == "VERTEX_SE2")
            {
                counter++;
                int id;
                Vector3d v;
                ls >> id >> v.x() >> v.y() >> v.z();
                os << "VERTEX_SE2 " << id + (i * offset) << " " << v.x() << " " << v.y() << " " << v.z() << endl;
            }
            else if(tag == "EDGE_SE2")
            {
                int id1, id2;
                Vector3d v;
                ls >> id1 >> id2 >> v.x() >> v.y() >> v.z();
                os << "EDGE_SE2 " << id1 + (i * offset) << " " << id2 + (i * offset) << " " << v.x() << " " << v.y() << " " << v.x()
                   << " 100 0 0 100 0 1000" << endl;
            }
            else
            {
                os << line << endl;
            }
        }
    }

    for(set<Edge*>::const_iterator it = _matches.begin(); it != _matches.end(); it++)
    {
        Edge* e = *it;
        Vector3d eye_tsf = utility::t2v(e->_transform);
        os << "EDGE_SE2 " << e->_from->_id + offset << " " << e->_to->_id << " " << eye_tsf.x() << " " << eye_tsf.y() << " " << eye_tsf.z()
           << " 100 0 0 100 0 1000" << endl;
    }
    os.close();
}


void GraphMatcher::circularMatch(Node *n1, Node *n2)
{
    NodeMap nm1 = _g1->nodeMap();
    NodeMap nm2 = _g2->nodeMap();

    for(NodeMap::const_iterator fit = nm1.begin(); fit != nm1.end(); ++fit)
    {
        Node* n1 = fit->second;
        for(NodeMap::const_iterator sit = nm2.begin(); sit != nm2.end(); ++sit)
        {
            Node* n2 = sit->second;
            Isometry2d transform = n2->_pose.inverse() * n1->_pose;
            double distance = utility::t2v(transform).squaredNorm();
            if(fabs(distance) < 10)
            {
                Edge* match = new Edge;
                match->_from = n2;
                match->_to = n1;
                match->_transform = transform;

                _matches.insert(match);
            }
        }
    }
}


void GraphMatcher::recursiveMatch(Node* root1, Node* root2)
{
    _visited1.insert(root1);
    _visited2.insert(root2);

    Edge* firstEdge = new Edge;
    firstEdge->_from = root1;
    firstEdge->_to = root2;
    _matches.insert(firstEdge);

    deque<Node*> q1, q2;
    q1.push_back(root1);
    q2.push_back(root2);

    while(!q1.empty() && !q2.empty())
    {
        Node* n1 = q1.front();
        q1.pop_front();
        Node* n2 = q2.front();
        q2.pop_front();

        const EdgeSet& n1_es = n1->edges();
        const EdgeSet& n2_es = n2->edges();

        for(EdgeSet::const_iterator fit = n1_es.begin(); fit != n1_es.end(); fit++)
        {
            Edge* e1 = *fit;
            Node* nn1 = (n1 == e1->_from) ? e1->_to : e1->_from;
            if(_visited1.count(nn1))
            {
                continue;
            }

            Node* best_match = 0;
            double best_chi = 1e5;
            for(EdgeSet::const_iterator sit = n2_es.begin(); sit != n2_es.end(); sit++)
            {
                Edge* e2 = *sit;
                Node* nn2 = (n2 == e2->_from) ? e2->_to : e2->_from;
                if(_visited2.count(nn2))
                {
                    continue;
                }

                Isometry2d et = n2->_pose.inverse() * n1->_pose;
                Isometry2d nn2_n1 = et * nn2->_pose;
                Isometry2d delta_m = nn1->_pose.inverse() * nn2_n1;
                Vector3d delta = utility::t2v(delta_m);
                cout << "NN1: " << nn1->_id << ", NN2: " << nn2->_id << endl;
                cout << "DELTA: " << delta.x() << " " << delta.y() << " " << delta.z() << endl;
                cout << "Chi2: " << delta.squaredNorm() << "; nn1: " << nn1->_id << "; nn2: " << nn2->_id << endl;

                q2.push_back(nn2);

                if(delta.squaredNorm() < best_chi)
                {
                    if(delta.squaredNorm() < 10)
                    {
                        best_match = nn2;
                        best_chi = delta.squaredNorm();

                        cout << "Best chi2: " << best_chi << "; nn1: " << nn1->_id << "; nn2: " << nn2->_id << endl;
                    }
                }
            }
            if(best_match)
            {
                Edge* e = new Edge;
                e->_from = nn1;
                e->_to = best_match;

                _matches.insert(e);
                _visited1.insert(nn1);
                _visited2.insert(best_match);
            }
            q1.push_back(nn1);
        }
        _visited1.insert(n1);
        _visited2.insert(n2);
    }
}
