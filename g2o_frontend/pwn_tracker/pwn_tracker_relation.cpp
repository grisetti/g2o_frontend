#include "pwn_tracker_relation.h"

namespace pwn_tracker {

  PwnTrackerRelation::PwnTrackerRelation(MapManager* manager, int id, IdContext* context):
    MapNodeBinaryRelation(manager, id, context){
    _nodes.resize(2);
    inliers = 0;
    error = 0;
  }
  
  void PwnTrackerRelation::serialize(ObjectData& data, IdContext& context){
    MapNodeBinaryRelation::serialize(data,context);
    data.setInt("inliers", inliers);
    data.setFloat("error", error);
  }
    //! boss deserialization
  void PwnTrackerRelation::deserialize(ObjectData& data, IdContext& context){
    MapNodeBinaryRelation::deserialize(data,context);
    inliers = data.getInt("inliers");
    error = data.getFloat("error");
  }

}
