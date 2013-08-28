#ifndef _BOSS_FRAME_H_
#define _BOSS_FRAME_H_
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include "g2o_frontend/boss/object_data.h"

// include this *before* any eigen class. It adds the serialization things to the fixed size eigen matrices
#include "eigen_boss_plugin.h" 

#include <Eigen/Geometry>
#include <set>

namespace boss {

//! a frame represents a transform in the system. This transform is relative to the 
//! parentFrame. If parentFrame is null, the transform is global.
class Frame: public Identifiable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::map<std::string, Frame*> ChildrenNameMap;
  Frame(const std::string& name_="", 
	const Eigen::Isometry3d& transform_=Eigen::Isometry3d::Identity(), 
	Frame* parentFrame_ = 0, int id=-1, IdContext* context = 0);
  virtual ~Frame();
  virtual void serialize(ObjectData& data, IdContext& context);
  virtual void deserialize(ObjectData& data, IdContext& context);
  virtual void deserializeComplete();
  inline const Eigen::Isometry3d& transform() const {return _transform;}
  inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
  
  inline const std::string& name() const {return _name;}
  void setName(const std::string& name_) {_name = name_;}
  std::string fullName() const;

  inline const Frame* parentFrame() const { return _parentFrame;}
  inline Frame* parentFrame() { return _parentFrame;}
  void setParentFrame(Frame* parentFrame_);
  inline const std::set<Frame*>& childrenFrames() const {return _childrenFrames;}
  Frame* childByName(const std::string& childrenName);
  const Frame* childByName(const std::string& childrenName) const;
  Eigen::Isometry3d transformTo(const Frame* base) const;
protected:
  Eigen::Isometry3d _transform;
  Frame* _parentFrame;
  std::set<Frame*> _childrenFrames;
  ChildrenNameMap _childrenNameMap;
  std::string _name;
};


}
#endif
