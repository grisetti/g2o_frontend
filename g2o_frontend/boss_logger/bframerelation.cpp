#include "bframerelation.h"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>

namespace boss {

  //! a frame relation is a transform (eventually with covariance), that specifies the relative transformation
  //! between two frames
  ReferenceFrameRelation::ReferenceFrameRelation(int id, IdContext* context):
    Identifiable(id,context){
    _fromReferenceFrame = 0;
    _toReferenceFrame = 0;
  }

  void ReferenceFrameRelation::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data, context);

    data.setPointer("fromReferenceFrame", _fromReferenceFrame);
    data.setPointer("toReferenceFrame", _toReferenceFrame);

    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
    _informationMatrix.toBOSS(data,"informationMatrix");
  }
  
  void ReferenceFrameRelation::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data, context);

    data.getReference("fromReferenceFrame").bind(_fromReferenceFrame);
    data.getReference("toReferenceFrame").bind(_toReferenceFrame);

    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
    _informationMatrix.fromBOSS(data,"informationMatrix");
  }

  void ReferenceFrameRelation::deserializeComplete(){
  }
  BOSS_REGISTER_CLASS(ReferenceFrameRelation);

} // end namespace
