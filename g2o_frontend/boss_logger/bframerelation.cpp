#include "bframerelation.h"

namespace boss {

//! a frame relation is a transform (eventually with covariance), that specifies the relative transformation
//! between two frames
FrameRelation::FrameRelation(int id, IdContext* context):
  Identifiable(id,context){
}

  void FrameRelation::serialize(ObjectData& /*data*/, IdContext& /*context*/){}
  void FrameRelation::deserialize(ObjectData& /*data*/, IdContext& /*context*/){}

} // end namespace
