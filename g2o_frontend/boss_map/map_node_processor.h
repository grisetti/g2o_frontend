#ifndef _BOSS_MAP_NODE_PROCESSOR_H_
#define _BOSS_MAP_NODE_PROCESSOR_H_

#include "map_manager.h"
#include "sensing_frame_node.h"
#include "imu_sensor.h"
#include "robot_configuration.h"
#include "stream_processor.h"

namespace boss_map {
  using namespace boss;
  using namespace std;

  class MapNodeProcessor: public StreamProcessor{
  public:
    MapNodeProcessor(MapManager* manager_,   RobotConfiguration* config_);
    template <class T>
    T* extractRelation(std::vector<MapNode*> nodes){
      if (nodes.size()==0)
	return 0;
      std::set<MapNodeRelation*>& relations = _manager->nodeRelations(nodes[0]);
      for (std::set<MapNodeRelation*>::iterator it=relations.begin(); it!=relations.end(); it++){
	MapNodeRelation* _rel=*it;
	T* rel = dynamic_cast<T*>(_rel);
	if (!rel)
	  continue;
	if (rel) {
	  for (size_t i=0; rel && i<nodes.size(); i++){
	    if(nodes[i]!=rel->nodes()[i])
	      rel = 0;
	  }
	  if (rel)
	    return rel;
	}
      }
      return 0;
    }
    virtual void process(Serializable* s);
  protected:
    virtual void processNode(MapNode* node) = 0;
    MapManager* _manager;
    RobotConfiguration* _config;
  };


  class ImuRelationAdder : public MapNodeProcessor{
  public:
    ImuRelationAdder(MapManager* manager_,   RobotConfiguration* config_);
    virtual void processNode(MapNode* node_);
    inline MapNodeUnaryRelation* lastImu() { return _lastIMU; }
  protected:
    MapNodeUnaryRelation* _lastIMU;
  };

  struct OdometryRelationAdder : public MapNodeProcessor{
  public:
    OdometryRelationAdder(MapManager* manager_,   RobotConfiguration* config_);
    virtual void processNode(MapNode* node_);
    inline MapNodeBinaryRelation* lastOdometry() { return _lastOdometry; }
  protected:
    MapNode* _previousNode;
    MapNodeBinaryRelation* _lastOdometry;
  };

}
#endif
