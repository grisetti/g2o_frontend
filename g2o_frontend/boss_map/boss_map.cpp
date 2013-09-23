#include "boss_map.h"
#include <stdexcept>
#include <iostream>

namespace boss_map {
  using namespace boss;
  using namespace std;

  /***************************************** MapNode *****************************************/
  MapNode::MapNode (MapManager* manager_,int id, IdContext* context) : Identifiable(id, context) {
    _manager = manager_;
    _level = 0;
  }
  
  //! boss serialization
  void MapNode::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    data.setPointer("manager", _manager);
    data.setInt("level", _level);
    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
  }
  
  //! boss deserialization
  void MapNode::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    data.getReference("manager").bind(_manager);
    _level = data.getInt("level");
    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
  }

  //! called when all links are resolved, adjusts the bookkeeping of the parents
  void MapNode::deserializeComplete(){
    assert(_manager);
    _manager->addNode(this);
  }

  
  std::vector<MapNode*> MapNode::parents() {
    std::vector<MapNodeRelation*> relations=parentRelations();
    std::set<MapNode*> pset;
    for (size_t i = 0; i<relations.size(); i++){
      MapNodeRelation* r=relations[i];
      for(size_t j=0; j<r->nodes().size(); j++){
	MapNode* n = r->nodes()[j];
	if (n!=this)
	  pset.insert(n);
      }
    }
    std::vector<MapNode*> parents_;
    for (std::set<MapNode*>::iterator it=pset.begin(); it!=pset.end(); it++)
      parents_.push_back(*it);
    return parents_;
  }
  
  std::vector<MapNodeRelation*> MapNode::parentRelations(){
    std::vector<MapNodeRelation*> ret;
    std::set<MapNodeRelation*>& rel=_manager->nodeRelations(this);
    for(std::set<MapNodeRelation*>::iterator it = rel.begin(); it!=rel.end(); it++){
      MapNodeRelation* r = *it;
      if (r->owner() && r->owner()->level()>level())
	ret.push_back(r);
    }
    return ret;
  }
  
  std::vector<MapNode*> MapNode::children() {
    std::vector<MapNodeRelation*> relations=childrenRelations();
    std::set<MapNode*> cset;
    for (size_t i = 0; i<relations.size(); i++){
      MapNodeRelation* r=relations[i];
      for(size_t j=0; j<r->nodes().size(); j++){
	MapNode* n = r->nodes()[j];
	if (n!=this)
	  cset.insert(n);
      }
    }
    std::vector<MapNode*> children_;
    for (std::set<MapNode*>::iterator it=cset.begin(); it!=cset.end(); it++)
      children_.push_back(*it);
    return children_;
  }
  
  std::vector<MapNodeRelation*> MapNode::childrenRelations(){
    std::vector<MapNodeRelation*> ret;
    std::set<MapNodeRelation*>& rel=_manager->ownedRelations(this);
    for(std::set<MapNodeRelation*>::iterator it = rel.begin(); it!=rel.end(); it++){
      MapNodeRelation* r = *it;
      ret.push_back(r);
    }
    return ret;
  }

  /***************************************** MapNodeAlias *****************************************/


  MapNodeAlias::MapNodeAlias(MapNode* original_, MapManager* manager, int id, IdContext* context):
    MapNode(manager, id, context){
    _original = original_;
  }
  const Eigen::Isometry3d& MapNodeAlias::transform() const {
    if (!_original){
      throw std::runtime_error("getting transform in alias with no concrete instance attached");
    }
    return _original->transform();
  }

  void MapNodeAlias::setTransform(const Eigen::Isometry3d& transform_) {
    if (!_original){
      throw std::runtime_error("setting transform in alias with no concrete instance attached");
    }
    _original->setTransform(transform_);
  }

  void MapNodeAlias::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("original", _original);
  }
  
  void MapNodeAlias::deserialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    data.getReference("original").bind(_original);
  }

  /***************************************** MapNodeRelation *****************************************/
  
  /**
     This class models a relation between one or more map nodes.
     A relation is a spatial constraint, that originates either by a sensor measurement
     or by a matching algorithm.
     In either case, this class should be specialized to retain the information
     about the specific result (e.g. number of inliers and so on).
   */
  MapNodeRelation::MapNodeRelation (MapManager* manager_, int id, IdContext* context): Identifiable(id,context){
    _manager=manager_;
    _generator = 0;
    _owner = 0;
  }
  
  void MapNodeRelation::serialize(ObjectData& data, IdContext& context) {
    Identifiable::serialize(data,context);
    data.setPointer("manager", _manager);
    data.setPointer("owner", _owner);
    data.setPointer("generator", _generator);
    ArrayData* adata = new ArrayData;
    for (size_t i=0; i<_nodes.size(); i++){
      adata->add(new PointerData(_nodes[i]));
    }
    data.setField("nodes", adata);
  }
  
  void MapNodeRelation::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    data.getReference("manager").bind(_manager);
    data.getReference("owner").bind(_owner);
    data.getReference("generator").bind(_generator);
    ArrayData* adata = dynamic_cast<ArrayData*>(data.getField("parents"));
    _nodes.resize(adata->size());
    for(size_t i = 0; i<adata->size(); i++){
      (*adata)[i].getReference().bind(_nodes[i]);
    }
  }

  void MapNodeRelation::deserializeComplete(){
    assert(_manager);
    _manager->addRelation(this);
  }

  /***************************************** MapNodeAliasRelation *****************************************/
  MapNodeAliasRelation::MapNodeAliasRelation (MapNode* owner_, MapManager* manager_, int id, IdContext* context): 
    MapNodeRelation(manager_, id,context){
    _owner = owner_;
    _manager=manager_;
    _generator = 0;
    _nodes.resize(2,0);
  }


  

  /***************************************** MapNodeBinaryRelation *****************************************/

  MapNodeBinaryRelation::MapNodeBinaryRelation(MapManager* manager, int id, IdContext* context):
    MapNodeRelation(manager, id, context){
    _nodes.resize(2,0);
    _transform.setIdentity();
  }

  //! boss serialization
  void MapNodeBinaryRelation::serialize(ObjectData& data, IdContext& context){
    MapNodeRelation::serialize(data,context);
    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
    _informationMatrix.toBOSS(data,"informationMatrix");
  }
  
  //! boss deserialization
  void MapNodeBinaryRelation::deserialize(ObjectData& data, IdContext& context){
    MapNodeRelation::deserialize(data,context);
    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
    _informationMatrix.fromBOSS(data,"informationMatrix");
  }

  /***************************************** MapNodeUnaryRelation *****************************************/

  MapNodeUnaryRelation::MapNodeUnaryRelation(MapManager* manager, int id, IdContext* context):
    MapNodeRelation(manager, id, context){
    _nodes.resize(1,0);
    _transform.setIdentity();
  }

  //! boss serialization
  void MapNodeUnaryRelation::serialize(ObjectData& data, IdContext& context){
    MapNodeRelation::serialize(data,context);
    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
    _informationMatrix.toBOSS(data,"informationMatrix");
  }
  
  //! boss deserialization
  void MapNodeUnaryRelation::deserialize(ObjectData& data, IdContext& context){
    MapNodeRelation::deserialize(data,context);
    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
    _informationMatrix.fromBOSS(data,"informationMatrix");
  }

  /***************************************** MapManager********************************/
  MapManager::MapManager(int id, IdContext* context) :Identifiable(id,context){}

  //! boss serialization
  void MapManager::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data, context);
  }

  //! boss deserialization

  void MapManager::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data, context);
  }

  bool MapManager::addNode(MapNode* n){
    std::set<MapNode*>::iterator it=_nodes.find(n);
    if (it!=_nodes.end())
      return false;
    //cerr << "inserting node " << n << endl;
    _nodes.insert(n);
    NodeInfo nInfo;
    _nodeInfos.insert(std::make_pair(n, nInfo));
    return true;
  }


  bool MapManager::addRelation(MapNodeRelation* relation) {
    std::set<MapNodeRelation*>::iterator it=_relations.find(relation);
    bool added = true;
    if (it == _relations.end()) {
      _relations.insert(relation);
    } else {
      removeRelation(relation);
      _relations.insert(relation);
      added = false;
    }
    //cerr << "inserting relation " << relation << endl;
    for(size_t i=0; i<relation->nodes().size(); i++){
      MapNode* n = relation->nodes()[i];
      std::map<MapNode*, NodeInfo>::iterator it=_nodeInfos.find(n);
      if (it == _nodeInfos.end()) {
	cerr << "node: " << n << endl;
	throw std::runtime_error("no node for relation");
      }
      it->second.relations.insert(relation);
    }
    if (relation->owner()){
      MapNode* n = relation->owner();
      std::map<MapNode*, NodeInfo>::iterator it=_nodeInfos.find(n);
      if (it == _nodeInfos.end())
	throw std::runtime_error("no owner for relation");
      it->second.ownerRelations.insert(relation);
    }
    return added;
  }

  bool MapManager::removeRelation(MapNodeRelation* relation){
    std::set<MapNodeRelation*>::iterator it=_relations.find(relation);
    if (it == _relations.end())
      return false;
    for(size_t i=0; i<relation->nodes().size(); i++){
      MapNode* n = relation->nodes()[i];
      std::map<MapNode*, NodeInfo>::iterator it=_nodeInfos.find(n);
      if (it == _nodeInfos.end()) {
	cerr << "node: " << n << endl;
	throw std::runtime_error("no node for relation");
      }
      it->second.relations.erase(relation);
    }
    if (relation->owner()){
      MapNode* n = relation->owner();
      std::map<MapNode*, NodeInfo>::iterator it=_nodeInfos.find(n);
      if (it == _nodeInfos.end())
	throw std::runtime_error("no owner for relation");
      it->second.ownerRelations.erase(relation);
    }
    _relations.erase(it);
    return true;
  }



  BOSS_REGISTER_CLASS(MapNode);
  BOSS_REGISTER_CLASS(MapNodeAlias);
  BOSS_REGISTER_CLASS(MapNodeRelation);
  BOSS_REGISTER_CLASS(MapNodeAliasRelation);
  BOSS_REGISTER_CLASS(MapNodeBinaryRelation);
  BOSS_REGISTER_CLASS(MapNodeUnaryRelation);
  BOSS_REGISTER_CLASS(MapManager);


}
