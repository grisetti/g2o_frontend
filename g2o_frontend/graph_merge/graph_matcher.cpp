#include "graph_matcher.h"
#include <iostream>

#define DEG2RAD(x) ((x) * 0.01745329251994329575)


using namespace Eigen;
using namespace g2o;
using namespace match_this;
using namespace std;


NodeMatcher::~NodeMatcher(){}


float IdealNodeMatcher::match(SE2& result, VertexSE2* v1, VertexSE2* v2)
{
    EdgeSE2* e = findEdge(v1,v2);
    if(!e)
    {
        return 1e9;
    }

    SE2 guess = v1->estimate().inverse()*v2->estimate();
    SE2 delta = e->measurement().inverse()*guess;
    Eigen::Vector3d vdelta = delta.toVector();
    Eigen::Matrix3d scale;
    scale <<1, 0, 0,
            0, 1, 0,
            0, 0, 10;
    double diff = vdelta.transpose()*scale*vdelta;
    double epsilon = 1;
    if(diff > epsilon)
        return 1e9;
    result = e->measurement();
    return 0.01;
}


EdgeSE2* IdealNodeMatcher::findEdge(VertexSE2* v1, VertexSE2* v2)
{
    for(EdgeSet::iterator it = _eset.begin(); it != _eset.end(); it++)
    {
        EdgeSE2* e = *it;
        if(e->vertex(0) == v1 && e->vertex(1) == v2)
        {
            return e;
        }
    }
    return 0;
}


RealNodeMatcher::RealNodeMatcher(ScanMatcher* sm, const float& max)
{
    _smatcher= sm;
    _mscore = max;
}


float RealNodeMatcher::match(SE2& result, VertexSE2 *v1, VertexSE2 *v2)
{
    _smatcher->match(v1, v2, _mscore);
    _smatcher->clear();

    if(_smatcher->getMatches().size() > 0)
    {
        ScanMatcherResult* res = (ScanMatcherResult*) _smatcher->getMatches()[0];
        Vector3f tsf = res->_transformation;
        float score = res->matchingScore();

        Isometry2f inner;
        inner = Rotation2Df(tsf.z());
        inner.translation() = Vector2f(tsf.x(), tsf.y());

        result = SE2(inner.cast<double>());
        _smatcher->clearMatchResults();
        return score;
    }
    else
    {
        cout << "no match" << endl;
        result = v1->estimate().toIsometry().inverse() * v2->estimate().toIsometry();

        _smatcher->clearMatchResults();
        return 0;
    }
}


GraphMatcher::GraphMatcher() {;}


GraphMatcher::GraphMatcher(NodeMatcher* m)
{
    if(m)
    {
        this->_matcher = m;
    }
}


NodeSet GraphMatcher::findNeighbors(g2o::HyperGraph::VertexIDMap* ref, const Isometry2d& transform, double epsilon)
{
    NodeSet result;
    result.clear();

    OptimizableGraph::VertexIDMap vertices = *ref;
    for(OptimizableGraph::VertexIDMap::const_iterator it = vertices.begin(); it != vertices.end(); it++)
    {
        VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
        Isometry2d v_tranform = v->estimate().toIsometry();
        Isometry2d candidate_tranform = transform.inverse() * v_tranform;
        Eigen::Matrix3d scale;
        scale <<
                 1, 0, 0,
                0, 1, 0,
                0, 0, 10;

        Eigen::Vector3d dv = utility::t2v(candidate_tranform);
        double distance = dv.transpose()*scale*dv;
        if(fabs(distance) < epsilon)
        {
            result.insert(v);
        }
    }
    return result;
}


void GraphMatcher::match(OptimizableGraph::VertexIDMap* ref, VertexSE2* first, const double& epsilon, const int& cnt)
{
    deque<VertexSE2*> queue;
    queue.push_back(first);
    Information& finfo = _currentInfo[first];
    finfo._parent = first;

    int iter = 0;
    while(!queue.empty() && iter != cnt)
    {   
        VertexSE2* current = queue.front();
        Information& cinfo = _currentInfo[current];
        queue.pop_front();

        // put all bloody childs in the queue
        for(OptimizableGraph::EdgeSet::iterator it = current->edges().begin(); it!=current->edges().end(); it++)
        {
            EdgeSE2* e = (EdgeSE2*)(*it);
            for(uint i = 0; i < e->vertices().size(); i++)
            {
                VertexSE2* other = (VertexSE2*) e->vertex(i);
                if(other == current)
                    continue;
                Information& otherInfo = _currentInfo[other];
                if(otherInfo._parent)
                    continue;
                otherInfo._parent = current;
                queue.push_back(other);
            }
        }

        // reassign the transforms if not the first node
        if(cinfo._parent != current)
        {
            Eigen::Isometry2d cT = cinfo._transform;
            VertexSE2* parent=cinfo._parent;

            Information& pinfo = _currentInfo[parent];
            Eigen::Isometry2d pT = pinfo._transform;
            Eigen::Isometry2d dt = pT.inverse()*cT;

            current->setEstimate(parent->estimate()*SE2(dt));

            NodeSet ref_neighbors = this->findNeighbors(ref, current->estimate().toIsometry(), epsilon);
            float bestScore = 1e9;
            SE2 bestTransform;
            VertexSE2* bestNeighbor = 0;
            for(NodeSet::iterator it = ref_neighbors.begin(); it != ref_neighbors.end(); it++)
            {
                VertexSE2* ref_neighbor = *it;
                SE2 transform;
                float score = _matcher->match(transform, current, ref_neighbor);
                if(score < bestScore)
                {
                    bestScore = score;
                    bestTransform = transform;
                    bestNeighbor = ref_neighbor;
                }
            }
            if(bestScore < 1)
            {
                EdgeSE2* newEdge = new EdgeSE2;
//                cerr << "bestScore: " << bestScore <<  " bestTransform: " << bestTransform.toVector().transpose() << endl;
                newEdge->setVertex(0,current);
                newEdge->setVertex(1,bestNeighbor);
                newEdge->setMeasurement(bestTransform);
                Matrix3d info = Matrix3d::Identity()*1000;
                newEdge->setInformation(info);
                _results.insert(newEdge);
//                cerr << "found closure edge between " << current->id() << " and " << bestNeighbor->id() << endl;
                current->setEstimate(bestNeighbor->estimate()*bestTransform.inverse());
            }
        }
        iter++;
    }
    cout << "iter: " << iter << endl;
}
