#ifndef _BOSS_LINKED_TREE_H_
#define _BOSS_LINKED_TREE_H_

#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/boss/serializable.h"
#include "g2o_frontend/boss/object_data.h"
#include <set>

namespace boss_map {
  using namespace boss;

//! generic tree like linked structure of strings
  template <class T>
  class LinkedTree: public T {
  public:
    typedef typename std::map<std::string, LinkedTree<T>*> ChildrenNameMap;
    typedef typename std::set< LinkedTree<T>* >  ChildrenSet;
    LinkedTree(const std::string& name_="", LinkedTree<T>* parent_ = 0, int id=-1, IdContext* context = 0);
    virtual ~LinkedTree();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    virtual void deserializeComplete();

    inline const std::string& name() const {return _name;}
  void setName(const std::string& name_) {_name = name_;}
    std::string path() const;
    
    inline const LinkedTree<T>* parent() const { return _parent;}
    inline LinkedTree<T>* parent() { return _parent;}
    void setParent(LinkedTree<T>* parent_);
    inline const ChildrenSet& children() const {return _children;}
    LinkedTree<T> * childByName(const std::string& childrenName);
    const LinkedTree<T> * childByName(const std::string& childrenName) const;
  protected:
    LinkedTree<T> * _parent;
    ChildrenSet _children;
    ChildrenNameMap _childrenNameMap;
    std::string _name;
  };

  template <class T>
  LinkedTree<T>::LinkedTree(const std::string& name_, LinkedTree* parent_ , int id, IdContext* context): T(id,context){
    _parent = parent_;
    _name = name_;
    if (_parent) {
      if (_parent->_childrenNameMap.find(_name)!=_parent->_childrenNameMap.end())
	throw std::runtime_error("error, adding frame name is already existing in tree");
      _parent->_children.insert(this);
      _parent->_childrenNameMap.insert(std::make_pair(name_,this));
    }
  }

  template <class T>
  LinkedTree<T>::~LinkedTree(){
    // detach from the parent
    if (_parent){
      _parent->_children.erase(this);
      _parent->_childrenNameMap.erase(this->name());
    }
    // detach from all children
    for (typename LinkedTree<T>::ChildrenSet::iterator it=_children.begin(); it!=_children.end(); it++){
      LinkedTree<T>* f=*it;
      f->_parent=0;
    }
  }
  
  
  template <class T>
  std::string LinkedTree<T>::path() const {
    if (! parent()) 
      return _name;
    return parent()->path() + "/" + name();
  }

  template <class T>
  LinkedTree<T>* LinkedTree<T>::childByName(const std::string& path){
    size_t i = path.find_first_of("/");
    std::string childName;
    std::string rest;
    if (i==std::string::npos)
      childName=path;
    else {
     childName = path.substr(0,i);
     rest=path.substr(i+1, std::string::npos);
    }
    typename LinkedTree<T>::ChildrenNameMap::iterator it=_childrenNameMap.find(childName);
    if(it == _childrenNameMap.end())
      return 0;
    LinkedTree<T>* childLinkedTree = it->second;
    if (rest.length()==0)
      return childLinkedTree;
    return childLinkedTree->childByName(rest);
  }

  template <class T>
  const LinkedTree<T>* LinkedTree<T>::childByName(const std::string& path) const {
    size_t i = path.find_first_of("/");
    std::string childName;
    std::string rest;
    if (i==std::string::npos)
      childName=path;
    else {
     childName = path.substr(0,i);
     rest=path.substr(i+1, std::string::npos);
    }
    typename LinkedTree<T>::ChildrenNameMap::const_iterator it=_childrenNameMap.find(childName);
    if(it == _childrenNameMap.end())
      return 0;
    const LinkedTree* child = it->second;
    if (rest.length()==0)
      return child;
    return child->childByName(rest);
  }


  template <class T>
  void LinkedTree<T>::setParent(LinkedTree<T>* parent_) {
    // detaches himself from the parent if existing
    if (_parent){
      _parent->_children.erase(this);
      _parent->_childrenNameMap.erase(this->name());
    }
    _parent = parent_;
    if (_parent){
      if (_parent->_childrenNameMap.find(_name)!=_parent->_childrenNameMap.end())
	throw std::runtime_error("error, adding frame name is already existing in tree");
      _parent->_children.insert(this);
      _parent->_childrenNameMap.insert(std::make_pair(this->name(),this));
    }
  }

  template <class T>
  void LinkedTree<T>::serialize(ObjectData& data, IdContext& context){
    T::serialize(data, context);
    data.setPointer("parent",_parent);
    data.setString("name", _name);
  }
  
  template <class T>
  void LinkedTree<T>::deserialize(ObjectData& data, IdContext& context){
    T::deserialize(data, context);
    _parent = 0;
    _name=data.getString("name");
    data.getReference("parent").bind(_parent);
  }

  template <class T>
  void LinkedTree<T>::deserializeComplete(){
    setParent(_parent);
  }

}
#endif
