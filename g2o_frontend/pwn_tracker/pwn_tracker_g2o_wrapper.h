#ifndef _PWN_TRACKER_G2O_WRAPPER_H_
#define _PWN_TRACKER_G2O_WRAPPER_H_

#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o_frontend/boss_map/boss_map_g2o_reflector.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "pwn_tracker.h"
#include "pwn_closer.h"

namespace pwn_tracker {
  using namespace pwn;
  using namespace boss;
  using namespace g2o;
  using namespace boss_map;
  using namespace pwn_tracker;

  class G2oWrapper{
  public:
    G2oWrapper(MapManager* manager);
    void optimize();
    SparseOptimizer * g2oInit();
    void save(const std::string& filename);
  
  protected:
    MapManager* manager;
    MapG2OReflector * reflector;
    SparseOptimizer * graph;
  };

}

#endif 
