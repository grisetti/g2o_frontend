#include "reference_frame.h"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>

namespace boss_map {
  using namespace std;
  using namespace boss;

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
    //cerr << "transform chain" << endl;
    //cerr << "left: " << endl;
    Eigen::Isometry3d t1=Eigen::Isometry3d::Identity();
    const ReferenceFrame* aux1 = this;
    while(aux1->parent()){
      t1=aux1->transform()*t1;
      //cerr << aux1->name() << endl;
      //cerr << t1.matrix() << endl << endl;
      aux1=static_cast<const ReferenceFrame*>(aux1->parent());
    }

    Eigen::Isometry3d t2=Eigen::Isometry3d::Identity();
    const ReferenceFrame* aux2 = base;
  //cerr << "transform chain" << endl;
  //cerr << "right: " << endl;
    while(aux2->parent()){
      t2=aux2->transform()*t2;
      //cerr << aux2->name() << endl;
      //cerr << t2.matrix() << endl << endl;
      aux2=static_cast<const ReferenceFrame*>(aux2->parent());
    }
    if (aux1!=aux2)
      throw std::runtime_error("the frames do not belong to the same tree");
    Eigen::Isometry3d ret = t2.inverse()*t1;
//cerr << "T:" << endl;
// cerr << ret.matrix() << endl;
return ret;
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
