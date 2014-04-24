#include "graph_simulator.h"
#include <iostream>

#define DEG2RAD(x) ((x) * 0.01745329251994329575)


using namespace Eigen;
using namespace g2o;
using namespace std;


SimEdge::SimEdge()
{
//    this->_from = 0;
//    this->_to = 0;
}


Isometry2d GraphSimulator::addNoise(const Isometry2d& lastPose, const Vector3d& noise)
{
    Vector3d last = utility::t2v(lastPose);
    Vector3d noisy(last.x() + Noise::gaussian(0.0, noise.x()), last.y() + Noise::gaussian(0.0, noise.y()), last.z() + Noise::gaussian(0.0, noise.z()));
    return utility::v2t(noisy);
}


SimNode GraphSimulator::generatePose(const SimNode& last, const Isometry2d& motion, const Vector3d& noise)
{
    SimNode nextPose;
    nextPose.id = last.id + 1;
    nextPose.real_pose = last.real_pose * motion;
    Isometry2d noiseMotion = addNoise(motion, noise);
    nextPose.noisy_pose = last.noisy_pose * noiseMotion;
    return nextPose;
}


Isometry2d GraphSimulator::generateMotion(int dir, double step)
{
    switch(dir)
    {
    case LEFT:
        return utility::v2t(Vector3d(step, 0, 0.5*M_PI));
    case RIGHT:
        return utility::v2t(Vector3d(step, 0, -0.5*M_PI));
    default:
        return utility::v2t(Vector3d(step, 0, -0.5*M_PI));
    }
}


void GraphSimulator::simulate(int samples, const Isometry2d& offset)
{
    this->simulate(samples, 1, 0, 1, offset);
}


void GraphSimulator::simulate(int samples, int trajectories, bool interClosures, bool lookForClosures, const Isometry2d& offset)
{
    // grid size
    int size = 50;

    // some parameters for the generation of the samples
    int forwardSteps = 3;
    double stepLength = 1.0;
    Isometry2d maxStepTransform(utility::v2t(Vector3d(forwardSteps * stepLength, 0, 0)));

    // Fake sensor for loop-closure detection
    double fov = (forwardSteps - 1) << 1;
    cout << "FOV: " << fov << endl;

    Vector2d grid(size >> 1, size >> 1);
    cout << "Grid: " << grid.x() << ", " << grid.y() << endl;
    Vector3d noise(0.05, 0.01, DEG2RAD(2.));

    VectorXd probLimits(POSSIBLE_MOTIONS);
    for(int i = 0; i < probLimits.size(); ++i)
    {
        probLimits[i] = (i + 1) / (double) POSSIBLE_MOTIONS;
    }

    Matrix3d covariance;
    covariance.fill(0.);
    covariance(0, 0) = noise[0] * noise[0];
    covariance(1, 1) = noise[1] * noise[1];
    covariance(2, 2) = noise[2] * noise[2];
    Matrix3d information = covariance.inverse();

    SimNode start;
    start.id = 0;
    start.real_pose = offset;
    start.noisy_pose = offset;

    for(short int k = 0; k < trajectories; ++k)
    {
        _trajectories.push_back(SimGraph());
        SimGraph& traj = _trajectories.back();

        Poses& poses = traj._poses;
        poses.clear();
        poses.push_back(start);

        // Nodes
        while((int) poses.size() < samples)
        {
            // go straight for some steps ...
            for(int i = 1; i < forwardSteps && (int) poses.size() < samples; ++i)
            {
                SimNode nextPose = generatePose(poses.back(), utility::v2t(Vector3d(stepLength, 0, 0)), noise);
                poses.push_back(nextPose);
            }
            if((int) poses.size() == samples)
            {
                break;
            }

            // ... now some other direction
            double uniform_value = Noise::uniform(0., 1.);
            int direction = 0;
            while(probLimits[direction] < uniform_value && direction + 1 < POSSIBLE_MOTIONS)
            {
                direction++;
            }
            Isometry2d nextMotion = generateMotion(direction, stepLength);
            SimNode nextPose = generatePose(poses.back(), nextMotion, noise);

            int* n = new int(k);
            prova.push_back(n);

            Isometry2d nextStepFinalPose = nextPose.real_pose * maxStepTransform;
            if(fabs(nextStepFinalPose.translation().x()) >= grid[0] || fabs(nextStepFinalPose.translation().y()) >= grid[1])
            {
                for(int i = 0; i < POSSIBLE_MOTIONS; ++i)
                {
                    nextMotion = generateMotion(i, stepLength);
                    nextPose = generatePose(poses.back(), nextMotion, noise);
                    nextStepFinalPose = nextPose.real_pose * maxStepTransform;
                    if(fabs(nextStepFinalPose.translation().x()) < grid[0] && fabs(nextStepFinalPose.translation().y()) < grid[1])
                    {
                        break;
                    }
                }
            }
            poses.push_back(nextPose);
        }
        cout << "Added Nodes" << endl;

        // Edges
        Edges& edges = traj._edges;
        edges.clear();
        for(size_t i = 1; i < poses.size(); ++i)
        {
            SimNode& prev = poses[i-1];
            SimNode& curr = poses[i];

            SimEdge* edge = new SimEdge;
            edge->from_id = prev.id;
            edge->to_id = curr.id;
            edge->real_transform = prev.real_pose.inverse() * curr.real_pose;
            edge->noisy_transform = prev.noisy_pose.inverse() * curr.noisy_pose;
            edge->information = information;

            edges.insert(edge);
            prev._connections.insert(edge);
        }
        cout << "Added Edges" << endl;

        // Loop Closures
        if(lookForClosures)
        {
            // Closures
            for(int i = poses.size() - 1; i >= 0; i--)
            {
                SimNode& sp = poses[i];
                for(int j = 0; j < i; j++)
                {
                    SimNode& candidate = poses[j];
                    Isometry2d transform = sp.real_pose.inverse() * candidate.real_pose;
                    double distance = utility::t2v(transform).squaredNorm();
                    if(fabs(distance) <= fov)
                    {
                        SimEdge* loopClosure = new SimEdge;
                        loopClosure->from_id = sp.id;
                        loopClosure->to_id = candidate.id;
                        loopClosure->real_transform = transform;
                        loopClosure->noisy_transform = transform;
                        loopClosure->information = information;

                        edges.insert(loopClosure);
                        sp._connections.insert(loopClosure);
                    }
                }
            }
        }
        traj.vec2map();
    }
    cout << "Added Loop Closures" << endl;

    // Inter Graph Closures
    if(interClosures)
    {
        Edges& closures = _closures;

        for(uint i = 1; i < _trajectories.size(); ++i)
        {
            SimGraph& t1 = _trajectories[i-1];
            SimGraph& t2 = _trajectories[i];
            const Poses& g1_poses = t1.poses();
            const Poses& g2_poses = t2.poses();
            for(uint i = 0; i < g2_poses.size(); ++i)
            {
                const SimNode& sp = g2_poses[i];
                for(uint j = 0; j < g1_poses.size(); ++j)
                {
                    const SimNode& candidate = g1_poses[j];
                    Isometry2d transform = sp.real_pose.inverse() * candidate.real_pose;
                    double distance = utility::t2v(transform).squaredNorm();
                    if(fabs(distance) <= fov)
                    {
                        SimEdge* graphClosure = new SimEdge;
                        graphClosure->from_id = sp.id;
                        graphClosure->to_id = candidate.id;
                        graphClosure->real_transform = transform;
                        graphClosure->noisy_transform = transform;

                        graphClosure->information = information;

                        closures.insert(graphClosure);
                    }
                }
            }
        }
        cout << "Added Inter Graph Closures" << endl;
    }
}


void SimGraph::vec2map()
{
    uint size = this->_poses.size();
    for(uint i = 0; i < size; i++)
    {
        SimNode sn = this->_poses[i];
        this->_nodes.insert(make_pair(sn.id, sn));
    }
}


NodeSet VirtualMatcher::findNeighbors(g2o::HyperGraph::VertexIDMap* ref, const Isometry2d& transform, double epsilon)
{
    NodeSet result;
    result.clear();

//    OptimizableGraph::VertexIDMap vertices = ref->vertices();
    OptimizableGraph::VertexIDMap vertices = *ref;
    for(OptimizableGraph::VertexIDMap::const_iterator it = vertices.begin(); it != vertices.end(); it++)
    {
        VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
        Isometry2d v_tranform = v->estimate().toIsometry();
        Isometry2d candidate_tranform = transform.inverse() * v_tranform;
        double distance = utility::t2v(candidate_tranform).squaredNorm();
        if(fabs(distance) < epsilon)
        {
            result.insert(v);
        }
    }
    return result;
}


void VirtualMatcher::tryMatch(VertexSE2* neighbor, VertexSE2* node, double& score, Isometry2d& tStar)
{
    Isometry2d neighbor_tsf = neighbor->estimate().toIsometry();
    Isometry2d node_tsf = node->estimate().toIsometry();

    tStar = node_tsf.inverse() * neighbor_tsf;
    score = utility::t2v(tStar).squaredNorm();
}


void VirtualMatcher::match(OptimizableGraph::VertexIDMap* ref, OptimizableGraph::VertexIDMap* curr, VertexSE2* first, double epsilon)
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
            double bestScore = epsilon;
            double score = bestScore;
            Isometry2d bestTransform = Isometry2d::Identity();
            for(NodeSet::iterator it = ref_neighbors.begin(); it != ref_neighbors.end(); it++) {
                VertexSE2* ref_neigbor = *it;
                this->tryMatch(ref_neigbor, node, score, bestTransform);
                if(score < bestScore) {
                    bestScore = score;
                    node->setEstimate(ref_neigbor->estimate().toIsometry() * bestTransform);
                    EdgeSE2* edge = new EdgeSE2;
                    edge->vertices()[0] = node;
                    edge->vertices()[1] = ref_neigbor;
                    edge->setMeasurement(node->estimate().toIsometry().inverse() * ref_neigbor->estimate().toIsometry());
                    edge->setInformation(Matrix3d::Identity());
                    this->_results.insert(edge);
                }
            }
        }

        // Look for all the children of the initial node in its graph
//        NodeSet node_neighbors = this->findNeighbors(curr, backup_transform, epsilon);
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





