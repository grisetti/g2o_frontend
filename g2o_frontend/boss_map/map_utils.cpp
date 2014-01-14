#include "map_utils.h"
#include <queue>

namespace boss_map {
  using namespace boss;

  NodeAcceptanceCriterion::NodeAcceptanceCriterion(MapManager* manager_, int id, boss::IdContext* context):
    Identifiable(id, context){
    _manager = manager_;
  }

  NodeAcceptanceCriterion::~NodeAcceptanceCriterion(){}


  void NodeAcceptanceCriterion::serialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::serialize(data,context);
    data.setPointer("manager",_manager);
  }

  void NodeAcceptanceCriterion::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
    data.getReference("manager").bind(_manager);
  }
    

  PoseAcceptanceCriterion::PoseAcceptanceCriterion(MapManager* manager_, int id, boss::IdContext* context): NodeAcceptanceCriterion(manager_, id, context){
    setReferencePose(Eigen::Isometry3d::Identity());
  }


  DistancePoseAcceptanceCriterion::DistancePoseAcceptanceCriterion(MapManager* manager_, int id, boss::IdContext* context): PoseAcceptanceCriterion(manager_, id, context){
    setTranslationalDistance(0.5);
    setRotationalDistance(0.5);
  }
  bool DistancePoseAcceptanceCriterion::accept(MapNode* n) {
    Eigen::Isometry3d _err=_invPose*n->transform();
    double dx = _err.translation().x();
    double dy = _err.translation().y();
    //hack
    if (dx*dx + dy*dy>_td2)
      return false;
    // if (_err.translation().squaredNorm()>_td2)
    //   return false;
    Eigen::AngleAxisd aa(_err.linear());
    if (fabs(aa.angle())>_rotationalDistance)
      return false;
    return true;
  }

  void DistancePoseAcceptanceCriterion::serialize(boss::ObjectData& data, boss::IdContext& context){
    PoseAcceptanceCriterion::serialize(data,context);
    data.setFloat("translationalDistance", _translationalDistance);
    data.setFloat("rotationalDistance", _rotationalDistance);
  }

  void DistancePoseAcceptanceCriterion::deserialize(boss::ObjectData& data, boss::IdContext& context){
    PoseAcceptanceCriterion::deserialize(data,context);
    setTranslationalDistance(data.getFloat("translationalDistance"));
    setRotationalDistance(data.getFloat("rotationalDistance"));
  }

  MahalanobisPoseAcceptanceCriterion::MahalanobisPoseAcceptanceCriterion(MapManager* manager_, int id, boss::IdContext* context): PoseAcceptanceCriterion(manager_, id, context){
    _info.setIdentity();
  }


  void MahalanobisPoseAcceptanceCriterion::serialize(boss::ObjectData& data, boss::IdContext& context){
    PoseAcceptanceCriterion::serialize(data,context);
    data.setDouble("distance", _distance);
  }

  void MahalanobisPoseAcceptanceCriterion::deserialize(boss::ObjectData& data, boss::IdContext& context){
    PoseAcceptanceCriterion::serialize(data,context);
    setDistance(data.getDouble("translationalDistance"));
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


  MapRelationSelector::MapRelationSelector(MapManager* manager_, int id, boss::IdContext* context):
    Identifiable(id, context) {_manager=manager_;}

  MapRelationSelector::~MapRelationSelector() {}

  void MapRelationSelector::serialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::serialize(data,context);
    data.setPointer("manager",_manager);
  }

  void MapRelationSelector::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
    data.getReference("manager").bind(_manager);
  }


  void makePartitions(std::vector<std::set<MapNode*> >& partitions,
		      std::set<MapNode*>& nodes, MapRelationSelector* relationSelector){
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
	  if (relationSelector && !relationSelector->accept(rel))
	    continue;
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

  BOSS_REGISTER_CLASS(DistancePoseAcceptanceCriterion);
  BOSS_REGISTER_CLASS(MahalanobisPoseAcceptanceCriterion);
}
