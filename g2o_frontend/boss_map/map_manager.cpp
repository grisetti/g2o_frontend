#include "map_manager.h"
#include <stdexcept>
#include <iostream>

namespace boss_map {
  using namespace boss;
  using namespace std;

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
    for (size_t i = 0; i<_actionHandlers.size(); i++){
      MapManagerActionHandler* handler = _actionHandlers[i];
      handler->nodeAdded(n);
    }
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
    
    if (added) {
      for (size_t i = 0; i<_actionHandlers.size(); i++){
	MapManagerActionHandler* handler = _actionHandlers[i];
	handler->relationAdded(relation);
      }
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
    for (size_t i = 0; i<_actionHandlers.size(); i++){
      MapManagerActionHandler* handler = _actionHandlers[i];
      handler->relationRemoved(relation);
    }
    
    return true;
  }

  MapManagerActionHandler::MapManagerActionHandler(MapManager* manager_){
    _manager = manager_;
    _manager->actionHandlers().push_back(this);
  }

  MapManagerActionHandler::~MapManagerActionHandler(){
    size_t i;
    for (i = 0;  i<_manager->actionHandlers().size() && _manager->actionHandlers()[i]!=this;  i++);
    if (i<_manager->actionHandlers().size())
      _manager->actionHandlers().erase(_manager->actionHandlers().begin()+i);
  }

  BOSS_REGISTER_CLASS(MapManager);

}
