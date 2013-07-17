#include "bframerelation.h"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>

namespace boss {

  //! a frame relation is a transform (eventually with covariance), that specifies the relative transformation
  //! between two frames
  FrameRelation::FrameRelation(int id, IdContext* context):
    Identifiable(id,context){
    _fromFrame = 0;
    _toFrame = 0;
  }

  void FrameRelation::serialize(ObjectData& data, IdContext& /*context*/){
    data.setPointer("fromFrame", _fromFrame);
    data.setPointer("toFrame", _toFrame);

    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
    _informationMatrix.toBOSS(data,"informationMatrix");
  }
  
  void FrameRelation::deserialize(ObjectData& data, IdContext& /*context*/){
    data.getPointer("fromFrame", _tempFromFrame);
    data.getPointer("toFrame", _tempToFrame);

    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
    _informationMatrix.fromBOSS(data,"informationMatrix");
  }

  void FrameRelation::deserializeComplete(){
    _fromFrame = static_cast<Frame*>(_tempFromFrame);
    _toFrame = static_cast<Frame*>(_tempToFrame);
  }
  BOSS_REGISTER_CLASS(FrameRelation);

} // end namespace
