#include "map_node_processor.h"
#include "g2o_frontend/basemath/bm_se3.h"

namespace boss_map {
  using namespace boss;
  
  void MapNodeProcessor::process(Serializable* s){
    MapNode* n = dynamic_cast<MapNode*>(s);
      if (n)
	processNode(n);
      else
	put(s);
  }


    MapNodeProcessor::MapNodeProcessor(MapManager* manager_,   RobotConfiguration* config_) {
      _manager = manager_;
      _config = config_;
    }

    void MapNodeProcessor::processNode(MapNode* n) {
      put(n);
    }

}
