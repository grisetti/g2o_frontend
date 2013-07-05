#include "bframe.h"
#include "g2o_frontend/boss/object_data.h"

namespace boss {

  Frame::Frame(Frame* parentFrame_ , int id, IdContext* context): Identifiable(id,context){
    _parentFrame = parentFrame_;
    if (_parentFrame)
      _parentFrame->_childrenFrames.insert(this);
    _transform.setIdentity();
  }

  Frame::~Frame(){
    // detach from the parent
    if (_parentFrame)
      _parentFrame->_childrenFrames.erase(this);

    // detach from all children
    for (std::set<Frame*>::iterator it=_childrenFrames.begin(); it!=_childrenFrames.end(); it++){
      Frame* f=*it;
      f->_parentFrame=0;
    }
  }

  void Frame::setParentFrame(Frame* parentFrame_) {
    // detaches himself from the parent if existing
    if (_parentFrame){
      _parentFrame->_childrenFrames.erase(this);
    }
    _parentFrame = parentFrame_;
    if (_parentFrame){
      _parentFrame->_childrenFrames.insert(this);
    }
  }


  void Frame::serialize(ObjectData& data, IdContext& /*context*/){
    data.setPointer("parentFrame",_parentFrame);
    ArrayData* array=new ArrayData();
    array->reserve(12);
    for (int r=0; r<3; r++)
      for(int c=0; c<4; c++)
	array->push_back(new NumberData(_transform.matrix()(r,c)));
    data.setField("transform",array);
  }
  
  void Frame::deserialize(ObjectData& data, IdContext& /*context*/){
    _parentFrame = 0;
    _tempParentFrame = 0;
    data.getPointer("parentFrame", _tempParentFrame);
    ArrayData* array = static_cast<ArrayData*>(data.getField("transform"));
    if(array->size()!=12){
      std::cerr << "AAAAA"; // throw an error
    }
    int k=0;
    for (int r=0; r<3; r++)
      for(int c=0; c<4; c++, k++)
	_transform.matrix()(r,c)=static_cast<NumberData*>( (*array)[k])->getDouble();
  }

  void Frame::deserializeComplete(){
    setParentFrame(static_cast<Frame*>(_tempParentFrame));
  }

}// end namespace
