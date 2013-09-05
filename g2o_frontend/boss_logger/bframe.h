#ifndef _BOSS_FRAME_H_
#define _BOSS_FRAME_H_
#include "blinked_tree.h"
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include "g2o_frontend/boss/object_data.h"

// include this *before* any eigen class. It adds the serialization things to the fixed size eigen matrices
#include "eigen_boss_plugin.h" 

#include <Eigen/Geometry>
#include <set>

namespace boss {

  class BaseFrame: public Identifiable{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BaseFrame (int id=-1, IdContext* context = 0);
    virtual ~BaseFrame();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    inline const Eigen::Isometry3d& transform() const {return _transform;}
    inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
  protected:
    Eigen::Isometry3d _transform;
  };


  //! a frame represents a transform in the system. This transform is relative to the 
  //! parentFrame. If parentFrame is null, the transform is global.
  class Frame: public LinkedTree<BaseFrame> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame(const std::string& name_="", 
	  const Eigen::Isometry3d& transform_=Eigen::Isometry3d::Identity(), 
	  Frame* parentFrame_ = 0, int id=-1, IdContext* context = 0);
    virtual ~Frame();

    Eigen::Isometry3d transformTo(const Frame* base) const;
    bool canTransformTo(const Frame* base) const;
    // casting sugar
    inline const Frame* parent() const { return (Frame*)_parent;}
    inline Frame* parent() { return (Frame*)_parent;}
    Frame * childByName(const std::string& childName) {return (Frame*) LinkedTree<BaseFrame>::childByName(childName);}
    const Frame * childByName(const std::string& childName) const {return (const Frame*) LinkedTree<BaseFrame>::childByName(childName);}

  };


}
#endif
