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
        return 0;

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
        return 0;
    result = e->measurement();
    return 1;
}


EdgeSE2* IdealNodeMatcher::findEdge(VertexSE2* v1, VertexSE2* v2)
{
    for(EdgeSet::iterator it = _eset.begin(); it != _eset.end(); it++) {
        EdgeSE2* e = *it;
        if(e->vertex(0) == v1 && e->vertex(1) == v2) {
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


//Lo scan matcher non trova un cavolo o sono io che non gli passo un cavolo?
float RealNodeMatcher::match(SE2& result, VertexSE2 *v1, VertexSE2 *v2)
{
    // the scan matcher should be given also other parameters (angular res and region size)
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

        Eigen::Vector3d dv=utility::t2v(candidate_tranform);
        double distance = dv.transpose()*scale*dv;
        if(fabs(distance) < epsilon)
        {
            result.insert(v);
        }
    }
    return result;
}


void GraphMatcher::match(OptimizableGraph::VertexIDMap* ref, VertexSE2* first, double epsilon)
{
    deque<VertexSE2*> queue;
    queue.push_back(first);
    Information& finfo = _currentInfo[first];
    finfo._parent = first;

    while(!queue.empty()) {
        VertexSE2* current = queue.front();
        //cerr << current->id() << endl;
        Information& cinfo = _currentInfo[current];
        queue.pop_front();

        // put all bloody childs in the queue
        for(OptimizableGraph::EdgeSet::iterator it = current->edges().begin(); it!=current->edges().end(); it++) {
            EdgeSE2* e = (EdgeSE2*)(*it);
            for(uint i = 0; i < e->vertices().size(); i++) {
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

        // reassign the trasforms if not the first node
        if(cinfo._parent !=current) {
            Eigen::Isometry2d cT = cinfo._transform;
            VertexSE2* parent=cinfo._parent;

            Information& pinfo = _currentInfo[parent];
            Eigen::Isometry2d pT = pinfo._transform;
            Eigen::Isometry2d dt = pT.inverse()*cT;

            current->setEstimate(parent->estimate()*SE2(dt));

            NodeSet ref_neighbors = this->findNeighbors(ref, current->estimate().toIsometry(), epsilon);
            double bestScore = 0;
            SE2 bestTransform;
            VertexSE2* bestNeighbor = 0;
            for(NodeSet::iterator it = ref_neighbors.begin(); it != ref_neighbors.end(); it++) {
                VertexSE2* ref_neighbor = *it;
                SE2 transform;
                float score = _matcher->match(transform, current, ref_neighbor);
                if(score>bestScore) {
                    bestScore = score;
                    bestTransform = transform;
                    bestNeighbor = ref_neighbor;
                }
            }
            if(bestScore>0) {
                EdgeSE2* newEdge = new EdgeSE2;
                newEdge->setVertex(0,current);
                newEdge->setVertex(1,bestNeighbor);
                newEdge->setMeasurement(bestTransform);
                Eigen::Matrix3d info=Eigen::Matrix3d::Identity()*1000;
                newEdge->setInformation(info);
                _results.insert(newEdge);
                cerr << "found closure edge between " << current->id() << " and " << bestNeighbor->id() << endl;
                current->setEstimate(bestNeighbor->estimate()*bestTransform.inverse());
            }
        }
    }
}

/*
void GraphMatcher::match(OptimizableGraph::VertexIDMap* ref, OptimizableGraph::VertexIDMap* curr, VertexSE2* first, double epsilon)
{
    // Setting the parent of the first node to itself
    if(this->_currentInfo.find(first) != this->_currentInfo.end())
    {
        Information& first_info = this->_currentInfo.find(first)->second;
        first_info._parent = first;
    }
    else
    {
        cout << "Could not find first node, exiting" << endl;
        return;
    }

    deque<VertexSE2*> queue;
    queue.push_back(first);

    while(!queue.empty())
    {
        VertexSE2* node = queue.front();
        queue.pop_front();
    cerr << "id: " << node->id() << endl;

        // Save the node isometry. it will be overwritten
        Isometry2d backup_transform = node->estimate().toIsometry();

        if(node != first)
        {
            Information node_info;
            if(this->_currentInfo.find(node) != this->_currentInfo.end()) {
                node_info = this->_currentInfo.find(node)->second;
            }
            else {
                cout << "Element not found, skipping" << endl;
            }

            if(node_info._parent) {
                // Move the node onto the other graph
                Information parent_info = this->_currentInfo.find(node_info._parent)->second;
                Isometry2d parent_tranform = parent_info._transform;
                Isometry2d current_transform = node_info._transform;

                Isometry2d delta = parent_tranform.inverse() * current_transform;
                node->setEstimate(parent_tranform * delta);
            }
            // Look for all the possible neighbors in the ref graph
            NodeSet ref_neighbors = this->findNeighbors(ref, node->estimate().toIsometry(), epsilon);

//            double bestScore = std::numeric_limits<double>::max();
            double bestScore = 0;
            SE2 bestTransform;
        VertexSE2* bestNeighbor = 0;
            for(NodeSet::iterator it = ref_neighbors.begin(); it != ref_neighbors.end(); it++) {
                VertexSE2* ref_neigbor = *it;
        SE2 transform;
        float score = _matcher->match(transform, ref_neigbor, node);
                if(score>bestScore) {
                    bestScore = score;
            bestTransform = transform;
            bestNeighbor = ref_neigbor;
                }
            }
        if (bestScore>0){
          EdgeSE2* newEdge;
          newEdge->setVertex(0,node);
          newEdge->setVertex(1,bestNeighbor);
          newEdge->setMeasurement(bestTransform);
          Eigen::Matrix3d info=Eigen::Matrix3d::Identity()*1000;
          newEdge->setInformation(info);
          _results.insert(newEdge);
        }
        }

        // Look for all the children of the initial node in its graph
        NodeSet node_neighbors = this->findNeighbors(curr, backup_transform, epsilon);
        for(NodeSet::iterator it = node_neighbors.begin(); it != node_neighbors.end(); it++) {
            VertexSE2* curr_neighbor = *it;
            if(this->_currentInfo.find(curr_neighbor) != this->_currentInfo.end()) {
                Information& curr_neighbor_info = this->_currentInfo.find(curr_neighbor)->second;
                if(!curr_neighbor_info._parent) {
                    curr_neighbor_info._parent = node;
                    queue.push_back(curr_neighbor);
                }
            }
        }
    }
}
*/
