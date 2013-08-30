#ifndef _BOSS_LINKED_TREE_H_
#define _BOSS_LINKED_TREE_H_
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include "g2o_frontend/boss/object_data.h"
#include <set>

namespace boss {

//! a frame represents a transform in the system. This transform is relative to the 
//! parentLinkedTree. If parentLinkedTree is null, the transform is global.
class LinkedTree: public Identifiable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::map<std::string, LinkedTree*> ChildrenNameMap;
  LinkedTree(const std::string& name_="", LinkedTree* parent_ = 0, int id=-1, IdContext* context = 0);
  virtual ~LinkedTree();
  virtual void serialize(ObjectData& data, IdContext& context);
  virtual void deserialize(ObjectData& data, IdContext& context);
  virtual void deserializeComplete();

  inline const std::string& name() const {return _name;}
  void setName(const std::string& name_) {_name = name_;}
  std::string path() const;

  inline const LinkedTree* parent() const { return _parent;}
  inline LinkedTree* parent() { return _parent;}
  void setParent(LinkedTree* parent_);
  inline const std::set<LinkedTree*>& children() const {return _children;}
  LinkedTree* childByName(const std::string& childrenName);
  const LinkedTree* childByName(const std::string& childrenName) const;
protected:
  LinkedTree* _parentLinkedTree;
  std::set<LinkedTree*> _childrenLinkedTrees;
  ChildrenNameMap _childrenNameMap;
  std::string _name;
};


}
#endif
