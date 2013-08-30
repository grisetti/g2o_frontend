#include "bframe.h"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>
namespace boss {
  using namespace std;

  BaseFrame::BaseFrame (int id, IdContext* context): Identifiable(id,context){
    _transform.setIdentity();
  }
  BaseFrame::~BaseFrame(){}

  void BaseFrame::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    Eigen::Quaterniond q(_transform.rotation());
      q.coeffs().toBOSS(data,"rotation");
      _transform.translation().toBOSS(data,"translation");
  }
  
  void BaseFrame::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
  }
  
  Frame::Frame(const std::string& name_, 
	       const Eigen::Isometry3d& transform_, 
	       Frame* parent_ , int id, IdContext* context): LinkedTree<BaseFrame>(name_,parent_,id,context){
    _transform = transform_;
  }

  Frame::~Frame(){
  }


  Eigen::Isometry3d Frame::transformTo(const Frame* base) const {
    // find the transform to the root;
    Eigen::Isometry3d t1=Eigen::Isometry3d::Identity();
    const Frame* aux1 = this;
    while(aux1->parent()){
      t1=t1*aux1->transform();
      aux1=static_cast<const Frame*>(aux1->parent());
    }

    Eigen::Isometry3d t2=Eigen::Isometry3d::Identity();
    const Frame* aux2 = base;
    while(aux2->parent()){
      t2=t2*aux2->transform();
      aux2=static_cast<const Frame*>(aux2->parent());
    }
    if (aux1!=aux2)
      throw std::runtime_error("the frames do not belong to the same tree");
    return t2.inverse()*t1;
  }


  BOSS_REGISTER_CLASS(Frame);

}// end namespace
