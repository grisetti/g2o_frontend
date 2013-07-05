#ifndef _BOSS_FRAME_H_
#define _BOSS_FRAME_H_
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <set>

namespace boss {

//! a frame represents a transform in the system. This transform is relative to the 
//! parentFrame. If parentFrame is null, the transform is global.
class Frame: public Identifiable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Frame(Frame* parentFrame_ = 0, int id=-1, IdContext* context = 0);
  virtual ~Frame();
  virtual void serialize(ObjectData& data, IdContext& context);
  virtual void deserialize(ObjectData& data, IdContext& context);
  virtual void deserializeComplete();
  inline const Eigen::Isometry3d& transform() {return _transform;}
  inline void setTransform(Eigen::Isometry3d& transform_) {_transform = transform_;}
  
  inline const Frame* parentFrame() const { return _parentFrame;}
  inline Frame* parentFrame() { return _parentFrame;}
  inline const std::set<Frame*>& childrenFrames() const {return _childrenFrames;}

protected:
  void setParentFrame(Frame* parentFrame_);
  Eigen::Isometry3d _transform;
  Frame* _parentFrame;
  std::set<Frame*> _childrenFrames;
private:
  Identifiable* _tempParentFrame;
};

}
#endif
