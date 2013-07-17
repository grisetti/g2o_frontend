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

  Eigen::Isometry3d Frame::transformTo(const Frame* base) const {
    // find the transform to the root;
    Eigen::Isometry3d t1=Eigen::Isometry3d::Identity();
    const Frame* aux1 = this;
    while(aux1->parentFrame()){
      t1=t1*aux1->transform();
      aux1=aux1->parentFrame();
    }

    Eigen::Isometry3d t2=Eigen::Isometry3d::Identity();
    const Frame* aux2 = base;
    while(aux2->parentFrame()){
      t2=t2*aux2->transform();
      aux2=aux2->parentFrame();
    }
    if (aux1!=aux2)
      throw std::runtime_error("the frames do not belong to the same tree");
    return t2.inverse()*t1;
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

  BOSS_REGISTER_CLASS(Frame);

}// end namespace
