#include "bframe.h"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>
namespace boss {
  using namespace std;

  Frame::Frame(const std::string& name_, 
	       const Eigen::Isometry3d& transform_, 
	       Frame* parentFrame_ , int id, IdContext* context): Identifiable(id,context){
    _parentFrame = parentFrame_;
    _name = name_;
    _transform = transform_;
    if (_parentFrame) {
      if (_parentFrame->_childrenNameMap.find(_name)!=_parentFrame->_childrenNameMap.end())
	throw std::runtime_error("error, adding frame name is already existing in frame");
      
      _parentFrame->_childrenFrames.insert(this);
      _parentFrame->_childrenNameMap.insert(std::make_pair(name_,this));
    }
  }

  Frame::~Frame(){
    // detach from the parent
    if (_parentFrame){
      _parentFrame->_childrenFrames.erase(this);
      _parentFrame->_childrenNameMap.erase(this->name());
    }
    // detach from all children
    for (std::set<Frame*>::iterator it=_childrenFrames.begin(); it!=_childrenFrames.end(); it++){
      Frame* f=*it;
      f->_parentFrame=0;
    }
  }

  std::string Frame::fullName() const {
    if (! parentFrame()) 
      return _name;
    return parentFrame()->fullName() + "/" + name();
  }

  Frame* Frame::childByName(const std::string& path){
    size_t i = path.find_first_of("/");
    std::string childName;
    std::string rest;
    if (i==std::string::npos)
      childName=path;
    else {
     childName = path.substr(0,i);
     rest=path.substr(i+1, std::string::npos);
    }
    ChildrenNameMap::iterator it=_childrenNameMap.find(childName);
    if(it == _childrenNameMap.end())
      return 0;
    Frame* childFrame = it->second;
    if (rest.length()==0)
      return childFrame;
    return childFrame->childByName(rest);
  }

  const Frame* Frame::childByName(const std::string& path) const {
    size_t i = path.find_first_of("/");
    std::string childName;
    std::string rest;
    if (i==std::string::npos)
      childName=path;
    else {
     childName = path.substr(0,i);
     rest=path.substr(i+1, std::string::npos);
    }
    ChildrenNameMap::const_iterator it=_childrenNameMap.find(childName);
    if(it == _childrenNameMap.end())
      return 0;
    const Frame* childFrame = it->second;
    if (rest.length()==0)
      return childFrame;
    return childFrame->childByName(rest);
  }


  void Frame::setParentFrame(Frame* parentFrame_) {
    // detaches himself from the parent if existing
    if (_parentFrame){
      _parentFrame->_childrenFrames.erase(this);
      _parentFrame->_childrenNameMap.erase(this->name());
    }
    _parentFrame = parentFrame_;
    if (_parentFrame){
      if (_parentFrame->_childrenNameMap.find(_name)!=_parentFrame->_childrenNameMap.end())
	throw std::runtime_error("error, adding frame name is already existing in frame");
      _parentFrame->_childrenFrames.insert(this);
      _parentFrame->_childrenNameMap.insert(std::make_pair(this->name(),this));
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


  void Frame::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data, context);
    data.setPointer("parentFrame",_parentFrame);
    data.setString("name", _name);
    
    Eigen::Quaterniond q(_transform.rotation());
    q.coeffs().toBOSS(data,"rotation");
    _transform.translation().toBOSS(data,"translation");
  }
  
  void Frame::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data, context);
    _parentFrame = 0;
    _name=data.getString("name");
    data.getReference("parentFrame").bind(_parentFrame);
    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
  }

  void Frame::deserializeComplete(){
    setParentFrame(_parentFrame);
  }

  BOSS_REGISTER_CLASS(Frame);

}// end namespace
