#pragma once

#include "g2o_frontend/boss_map/map_manager.h"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/pwn_core/frame.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include  "pwn_tracker_frame.h"

namespace pwn_tracker{

using namespace std;
using namespace boss;
using namespace boss_map;
using namespace pwn;


  struct PwnTrackerRelation: public MapNodeBinaryRelation{
    PwnTrackerRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    inline PwnTrackerFrame* from() { return static_cast<PwnTrackerFrame*>(_nodes[0]); }
    inline PwnTrackerFrame* to()   { return static_cast<PwnTrackerFrame*>(_nodes[1]); }
    void setFrom(PwnTrackerFrame* from_) {_nodes[0] = from_; }
    void setTo(PwnTrackerFrame* to_) {_nodes[1] = to_; }
    int inliers;
    float error;
  };

}
