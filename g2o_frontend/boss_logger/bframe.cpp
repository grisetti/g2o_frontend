#include "bframe.h"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>
namespace boss {
  using namespace std;

  BaseReferenceFrame::BaseReferenceFrame (int id, IdContext* context): Identifiable(id,context){
    _transform.setIdentity();
  }
  BaseReferenceFrame::~BaseReferenceFrame(){}

  void BaseReferenceFrame::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    Eigen::Quaterniond q(_transform.rotation());
      q.coeffs().toBOSS(data,"rotation");
      _transform.translation().toBOSS(data,"translation");
  }
  
  void BaseReferenceFrame::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    Eigen::Quaterniond q;
    q.coeffs().fromBOSS(data,"rotation");
    _transform.translation().fromBOSS(data,"translation");
    _transform.linear()=q.toRotationMatrix();
  }
  
  ReferenceFrame::ReferenceFrame(const std::string& name_, 
	       const Eigen::Isometry3d& transform_, 
	       ReferenceFrame* parent_ , int id, IdContext* context): LinkedTree<BaseReferenceFrame>(name_,parent_,id,context){
    _transform = transform_;
  }

  ReferenceFrame::~ReferenceFrame(){
  }


  Eigen::Isometry3d ReferenceFrame::transformTo(const ReferenceFrame* base) const {
    // find the transform to the root;
    Eigen::Isometry3d t1=Eigen::Isometry3d::Identity();
    const ReferenceFrame* aux1 = this;
    while(aux1->parent()){
      t1=t1*aux1->transform();
      aux1=static_cast<const ReferenceFrame*>(aux1->parent());
    }

    Eigen::Isometry3d t2=Eigen::Isometry3d::Identity();
    const ReferenceFrame* aux2 = base;
    while(aux2->parent()){
      t2=t2*aux2->transform();
      aux2=static_cast<const ReferenceFrame*>(aux2->parent());
    }
    if (aux1!=aux2)
      throw std::runtime_error("the frames do not belong to the same tree");
    return t2.inverse()*t1;
  }

  bool ReferenceFrame::canTransformTo(const ReferenceFrame* base) const {
    // find the transform to the root;
    const ReferenceFrame* aux1 = this;
    while(aux1->parent()){
      aux1=static_cast<const ReferenceFrame*>(aux1->parent());
    }

    const ReferenceFrame* aux2 = base;
    while(aux2->parent()){
      aux2=static_cast<const ReferenceFrame*>(aux2->parent());
    }
    if (aux1!=aux2)
      return false;
    return true;
  }


  BOSS_REGISTER_CLASS(ReferenceFrame);

}// end namespace
