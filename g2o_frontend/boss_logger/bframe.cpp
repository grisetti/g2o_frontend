#include "bframe.h"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>
namespace boss {

  Frame::Frame(Frame* parentFrame_ , int id, IdContext* context): Identifiable(id,context){
    _parentFrame = parentFrame_;
    if (_parentFrame)
      _parentFrame->_childrenFrames.insert(this);
    _transform.setIdentity();
  }

  Frame::~Frame(){
    // detach from the parent
    if (_parentFrame)
      _parentFrame->_childrenFrames.erase(this);

    // detach from all children
    for (std::set<Frame*>::iterator it=_childrenFrames.begin(); it!=_childrenFrames.end(); it++){
      Frame* f=*it;
      f->_parentFrame=0;
    }
  }

  void Frame::setParentFrame(Frame* parentFrame_) {
    // detaches himself from the parent if existing
    if (_parentFrame){
      _parentFrame->_childrenFrames.erase(this);
    }
    _parentFrame = parentFrame_;
    if (_parentFrame){
      _parentFrame->_childrenFrames.insert(this);
    }
  }


  void Frame::serialize(ObjectData& data, IdContext& /*context*/){
    data.setPointer("parentFrame",_parentFrame);

    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
  }
  
  void Frame::deserialize(ObjectData& data, IdContext& /*context*/){
    _parentFrame = 0;
    _tempParentFrame = 0;
    data.getPointer("parentFrame", _tempParentFrame);

    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
  }

  void Frame::deserializeComplete(){
    setParentFrame(static_cast<Frame*>(_tempParentFrame));
  }

}// end namespace
