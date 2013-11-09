#ifndef _BOSS_MAP_MANAGER_H_
#define _BOSS_MAP_MANAGER_H_

#include "boss_map.h"

namespace boss_map {
  using namespace boss;
  
  class MapManagerActionHandler{
  public:
    MapManagerActionHandler(MapManager* _manager);
    virtual ~MapManagerActionHandler();
    inline MapManager* mapManager() {return _manager;}
    virtual void nodeAdded(MapNode* n)=0;
    virtual void nodeRemoved(MapNode* n)=0;
    virtual void relationAdded(MapNodeRelation* r)=0;
    virtual void relationRemoved(MapNodeRelation* r)=0;
  protected:
    MapManager* _manager;
  };

  /***************************************** MapManager********************************/
  class MapManager: public Identifiable {
  protected:
    struct NodeInfo{
      std::set<MapNodeRelation*> relations;
      std::set<MapNodeRelation*> ownerRelations;
     };
  public:
    MapManager(int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);

    bool addNode(MapNode* n);
    bool removeNode(MapNode* n);
    bool addRelation(MapNodeRelation* relation);
    bool removeRelation(MapNodeRelation* relation);

    inline std::set<MapNode*>& nodes() {return _nodes;}
    inline std::set<MapNodeRelation*>& relations() {return _relations;}
    
    inline std::set<MapNodeRelation*>& nodeRelations(MapNode* n) {return _nodeInfos[n].relations;}
    inline std::set<MapNodeRelation*>& ownedRelations(MapNode* n) {return _nodeInfos[n].ownerRelations;}

    inline std::vector<MapManagerActionHandler*>& actionHandlers() { return _actionHandlers; }

  protected:
    std::vector<MapManagerActionHandler*> _actionHandlers;
    std::set<MapNode*> _nodes;
    std::set<MapNodeRelation*> _relations;
    std::map<MapNode*, NodeInfo> _nodeInfos;
  };

}


#endif
