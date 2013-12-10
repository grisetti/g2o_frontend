#pragma once

#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/pwn_core/frame.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/pwn_core/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o_frontend/boss_map_building/boss_map_g2o_reflector.h"
#include "g2o_frontend/boss_map/map_utils.h"
//#include "pwn_tracker.h"
//#include "pwn_closer.h"

namespace boss_map {
  using namespace pwn;
  using namespace boss;
  using namespace g2o;
  using namespace boss_map;
  using namespace boss_map_building;

  class G2oWrapper{
  public:
    G2oWrapper(MapManager* manager);
    void optimize();
    SparseOptimizer * g2oInit();
    void save(const std::string& filename);
    MapRelationSelector* _selector;
  protected:
    MapManager* manager;
    MapG2OReflector * reflector;
    SparseOptimizer * graph;
  };

}
