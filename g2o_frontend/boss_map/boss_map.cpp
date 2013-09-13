#include "boss_map.h"
#include <stdexcept>
namespace boss {

  /***************************************** MapNode *****************************************/
  MapNode::MapNode (MapManager* manager_,int id, IdContext* context) : Identifiable(id, context) {
    _manager = manager_;
  }
  
  //! boss serialization
  void MapNode::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    data.setPointer("manager", _manager);
    ArrayData* adata = new ArrayData;
    for (size_t i=0; i<_parents.size(); i++){
      adata->add(new PointerData(_parents[i]));
    }
    data.setField("parents", adata);
    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
  }
  
  //! boss deserialization
  void MapNode::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    data.getReference("manager").bind(_manager);
    ArrayData* adata = dynamic_cast<ArrayData*>(data.getField("parents"));
    _parents.resize(adata->size());
    for(size_t i = 0; i<adata->size(); i++){
      (*adata)[i].getReference().bind(_parents[i]);
    }
    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
  }

  //! called when all links are resolved, adjusts the bookkeeping of the parents
  void MapNode::deserializeComplete(){
    assert(_manager);
    _manager->addNode(this);
    for(size_t i = 0; i<_parents.size(); i++){
      _manager->addParentToNode(this,_parents[i]);
    }
  }
  
  //! climbs up in the hierarchy of parents by counting the level. The hierarchical map, seen vertically is a DAG
  int MapNode::level(){
    int l=0;
    MapNode* n=this;
    while(n && n->_parents.size()){
      l++;
      n = n->_parents[0];
    }
    return l;
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
  }
  
  void MapNodeRelation::serialize(ObjectData& data, IdContext& context) {
    Identifiable::serialize(data,context);
    data.setPointer("manager", _manager);
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



  /***************************************** MapNodeRelation *****************************************/
  MapNodeCollection::MapNodeCollection (MapManager* manager, int id, IdContext* context): MapNode(manager,id,context){
    _gauge=0;
  }

  void MapNodeCollection::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("gauge",_gauge);
    ArrayData* adata = new ArrayData;
    for (size_t i=0; i<_children.size(); i++){
      adata->add(new PointerData(_children[i]));
    }
    data.setField("children", adata);

    ArrayData* adata2 = new ArrayData;
    for (size_t i=0; i<_internalRelations.size(); i++){
      adata2->add(new PointerData(_internalRelations[i]));
    }
    data.setField("internalRelations", adata2);

    ArrayData* adata3 = new ArrayData;
    for (size_t i=0; i<_internalRelations.size(); i++){
      adata3->add(new PointerData(_internalRelations[i]));
    }
    data.setField("starRelations", adata3);

  }
  void MapNodeCollection::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
    data.getReference("gauge").bind(_gauge);

    ArrayData* adata = dynamic_cast<ArrayData*>(data.getField("children"));
    _children.resize(adata->size());
    for(size_t i = 0; i<adata->size(); i++){
      (*adata)[i].getReference().bind(_children[i]);
    }

    ArrayData* adata2 = dynamic_cast<ArrayData*>(data.getField("internalRelations"));
    _internalRelations.resize(adata2->size());
    for(size_t i = 0; i<adata->size(); i++){
      (*adata2)[i].getReference().bind(_internalRelations[i]);
    }

    ArrayData* adata3 = dynamic_cast<ArrayData*>(data.getField("starRelations"));
    _starRelations.resize(adata3->size());
    for(size_t i = 0; i<adata->size(); i++){
      (*adata3)[i].getReference().bind(_starRelations[i]);
    }

  }

  void MapNodeCollection::deserializeComplete(){
    MapNode::deserializeComplete();
  }
  

  /***************************************** SensingFrame *****************************************/

  SensingFrame::SensingFrame (MapManager* manager, int id, IdContext* context):
    MapNode(manager, id, context){
  }
  
  void SensingFrame::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    ArrayData* adata = new ArrayData;
    for (size_t i=0; i<_sensorDatas.size(); i++){
      adata->add(new PointerData(_sensorDatas[i]));
    }
    data.setField("sensorDatas", adata);
  }

  void SensingFrame::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
    ArrayData* adata = dynamic_cast<ArrayData*>(data.getField("sensorDatas"));
    _sensorDatas.resize(adata->size());
    for(size_t i = 0; i<adata->size(); i++){
      (*adata)[i].getReference().bind(_sensorDatas[i]);
    }
  }

  ReferenceFrame* SensingFrame::robotFrame(){
    if (! _sensorDatas.size())
      return 0;
    return _sensorDatas[0]->robotReferenceFrame();
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
    _nodes.insert(n);
    _nodesRelationMap.insert(make_pair(n, std::set<MapNodeRelation*>()));
    return true;
  }

  bool MapManager::addParentToNode(MapNode* child, MapNodeCollection* parent){
    std::set<MapNode*>::iterator ct=_nodes.find(child);
    std::set<MapNode*>::iterator pt=_nodes.find(parent);
    if(ct == _nodes.end() || pt == _nodes.end())
      return false;
    // TODO: check for consistency
    size_t i;
    for(i=0; i<parent->children().size(); i++){
      if (parent->children()[i]==child)
	break;
    }
    if (i==parent->children().size())
      parent->children().push_back(child);

    for(i=0; i<child->parents().size(); i++){
      if (child->parents()[i]==child)
	break;
    }
    if (i==child->parents().size())
      child->parents().push_back(parent);
    return true;
  }

  bool MapManager::removeParentFromNode(MapNode* child, MapNodeCollection* parent){
    std::set<MapNode*>::iterator ct=_nodes.find(child);
    std::set<MapNode*>::iterator pt=_nodes.find(parent);
    if(ct == _nodes.end() || pt == _nodes.end())
      return false;
    // TODO: check for consistency
    size_t i;
    for(i=0; i<parent->children().size(); i++){
      if (parent->children()[i]==child)
	break;
    }
    if (i==parent->children().size())
      return false;
    parent->children().erase(parent->children().begin()+i);
    for(i=0; i<child->parents().size(); i++){
      if (child->parents()[i]==child)
	break;
    }
    if (i==child->parents().size())
      return false;
    child->parents().erase(child->parents().begin()+i);
    return true;
  }


  bool MapManager::addRelation(MapNodeRelation* relation) {
    std::set<MapNodeRelation*>::iterator it=_relations.find(relation);
    if (it != _relations.end())
      return false;
    _relations.insert(relation);
    for(size_t i=0; i<relation->nodes().size(); i++){
      MapNode* n = relation->nodes()[i];
      std::map<MapNode*, std::set<MapNodeRelation*> >::iterator it = _nodesRelationMap.find(n);
      if (it==_nodesRelationMap.end()){
	throw std::runtime_error("addRelation, no node exising");
	std::set<MapNodeRelation*> s;
	s.insert(relation);
	_nodesRelationMap.insert(make_pair(n,s));
      } else {
	it->second.insert(relation);
      }
    }
    return true;
  }
  
  bool MapManager::removeRelation(MapNodeRelation* relation){
    std::set<MapNodeRelation*>::iterator it=_relations.find(relation);
    if (it == _relations.end())
      return false;
    _relations.erase(it);
    return true;
  }


  BOSS_REGISTER_CLASS(MapNode);
  BOSS_REGISTER_CLASS(MapNodeCollection);
  BOSS_REGISTER_CLASS(MapNodeRelation);
  BOSS_REGISTER_CLASS(SensingFrame);
  BOSS_REGISTER_CLASS(MapNodeBinaryRelation);
  BOSS_REGISTER_CLASS(MapNodeUnaryRelation);
  BOSS_REGISTER_CLASS(MapManager);


}
