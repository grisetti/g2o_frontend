#include "boss_map_utils.h"
#include <queue>

namespace boss_map {

  NodeAcceptanceCriterion::NodeAcceptanceCriterion(MapManager* manager_){
    _manager = manager_;
  }
  NodeAcceptanceCriterion::~NodeAcceptanceCriterion(){}



  PoseAcceptanceCriterion::PoseAcceptanceCriterion(MapManager* manager_): NodeAcceptanceCriterion(manager_){
    setReferencePose(Eigen::Isometry3d::Identity());
  }


  DistancePoseAcceptanceCriterion::DistancePoseAcceptanceCriterion(MapManager* manager_): PoseAcceptanceCriterion(manager_){
    setTranslationalDistance(0.5);
    setRotationalDistance(0.5);
  }
  bool DistancePoseAcceptanceCriterion::accept(MapNode* n) {
    Eigen::Isometry3d _err=_invPose*n->transform();
    if (_err.translation().squaredNorm()>_td2)
      return false;
    Eigen::AngleAxisd aa(_err.linear());
    if (fabs(aa.angle())>_rotationalDistance)
      return false;
    return true;
  }

  MahalanobisPoseAcceptanceCriterion::MahalanobisPoseAcceptanceCriterion(MapManager* manager_): PoseAcceptanceCriterion(manager_){
    _info.setIdentity();
  }
  bool MahalanobisPoseAcceptanceCriterion::accept(MapNode* n) {
    Eigen::Isometry3d err=_invPose*n->transform();
    Vector6d v=t2v(err);
    double d=v.transpose()*_info*v;
    return d<_distance;
  }



  void selectNodes(std::set<MapNode*>& nodes, NodeAcceptanceCriterion* criterion){
    MapManager* manager = criterion->manager();
    for (std::set<MapNode*>::iterator it=manager->nodes().begin(); it!=manager->nodes().end(); it++){
      MapNode* n = *it;
      if (criterion->accept(n))
	nodes.insert(*it);
    }
  }

  void extractInternalRelations(std::set<MapNodeRelation*>& internalRelations, 
				std::set<MapNode*>& nodes, 
				MapManager* manager){
    for (std::set<MapNode*>::iterator it=manager->nodes().begin(); it!=manager->nodes().end(); it++){
      MapNode* n = *it;
      std::set<MapNodeRelation*>& relations = manager->nodeRelations(n);
      for (std::set<MapNodeRelation*>::iterator rt=relations.begin(); rt!=relations.end(); rt++){
	bool add = true;
	MapNodeRelation* rel=*rt;
	for (size_t i = 0; i<rel->nodes().size(); i++){
	  MapNode* n2=rel->nodes()[i];
	  if (nodes.find(n2)==nodes.end()){
	    add = false;
	    break;
	  }
	}
	if (add)
	  internalRelations.insert(rel); 
      }
    }
  }

  void makePartitions(std::vector<std::set<MapNode*> >& partitions,
		      std::set<MapNode*>& nodes){
    std::set<MapNode*> openNodes = nodes;
    while(! openNodes.empty()){
      std::queue<MapNode*> queue;
      std::set<MapNode*> currentPartition;
      MapNode* first=*openNodes.begin();
      currentPartition.insert(first);
      queue.push(first);
      openNodes.erase(first);
      while(! queue.empty()){
	MapNode* n=queue.front();
	queue.pop();
	std::set<MapNodeRelation*>& relations=n->manager()->nodeRelations(n);
	for(std::set<MapNodeRelation*>::iterator it=relations.begin(); it!=relations.end(); it++){
	  MapNodeRelation* rel=*it;
	  for (size_t i=0; i<rel->nodes().size(); i++){
	    MapNode* nOther = rel->nodes()[i];
	    // if the node has already been visited or is not in the set, skip)
	    if (!openNodes.count(nOther))
	      continue;
	    queue.push(nOther);
	    openNodes.erase(nOther);
	    currentPartition.insert(nOther);
	  }
	}
      }
      partitions.push_back(currentPartition);
    }
  }

}
