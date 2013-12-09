#ifndef _BOSS_FRAME_H_
#define _BOSS_FRAME_H_

#include "linked_tree.h"
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include "g2o_frontend/boss/object_data.h"

// include this *before* any eigen class. It adds the serialization things to the fixed size eigen matrices
#include "eigen_boss_plugin.h" 

#include <Eigen/Geometry>
#include <set>

namespace boss_map {
  using namespace boss;

  class BaseReferenceFrame: public Identifiable{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BaseReferenceFrame (int id=-1, IdContext* context = 0);
    virtual ~BaseReferenceFrame();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    inline const Eigen::Isometry3d& transform() const {return _transform;}
    inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
  protected:
    Eigen::Isometry3d _transform;
  };


  //! a frame represents a transform in the system. This transform is relative to the 
  //! parentReferenceFrame. If parentReferenceFrame is null, the transform is global.
  class ReferenceFrame: public LinkedTree<BaseReferenceFrame> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ReferenceFrame(const std::string& name_="", 
	  const Eigen::Isometry3d& transform_=Eigen::Isometry3d::Identity(), 
	  ReferenceFrame* parentReferenceFrame_ = 0, int id=-1, IdContext* context = 0);
    virtual ~ReferenceFrame();

    Eigen::Isometry3d transformTo(const ReferenceFrame* base) const;
    bool canTransformTo(const ReferenceFrame* base) const;
    // casting sugar
    inline const ReferenceFrame* parent() const { return (ReferenceFrame*)_parent;}
    inline ReferenceFrame* parent() { return (ReferenceFrame*)_parent;}
    ReferenceFrame * childByName(const std::string& childName) {return (ReferenceFrame*) LinkedTree<BaseReferenceFrame>::childByName(childName);}
    const ReferenceFrame * childByName(const std::string& childName) const {return (const ReferenceFrame*) LinkedTree<BaseReferenceFrame>::childByName(childName);}
    
    inline bool isChildrenOf(const ReferenceFrame* p) const {
      const ReferenceFrame* f=this;
      while (f && f!=p){ f = (f->parent()); }
      return f!=0;
    }
  };


}
#endif
