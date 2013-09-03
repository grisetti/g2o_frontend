#ifndef _BOSS_FRAME_RELATION_H_
#define _BOSS_FRAME_RELATION_H_
#include "bframe.h"

namespace boss {

  //! a frame relation is a transform (eventually with covariance), that specifies the relative transformation
  //! between two frames
  class FrameRelation: public boss::Identifiable{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    FrameRelation(int id=-1, boss::IdContext* context = 0);
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserializeComplete();

    inline const Eigen::Isometry3d& transform() {return _transform;}
    inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
  
    inline const Eigen::Matrix<double, 6,6>& informationMatrix() {return _informationMatrix;}
    inline void setInformationMatrix(const Eigen::Matrix<double, 6,6>& informationMatrix_) {_informationMatrix=informationMatrix_;}

    inline const Frame* fromFrame() const { return _fromFrame;}
    inline Frame* fromFrame() { return _fromFrame;}
    inline void setFromFrame(Frame* from_) {_fromFrame = from_;}


    inline const Frame* toFrame() const { return _toFrame;}
    inline Frame* toFrame() { return _toFrame;}
    inline void setToFrame(Frame* to_) {_toFrame = to_;}

  protected:
    Eigen::Isometry3d _transform;
    Eigen::Matrix<double, 6,6> _informationMatrix;
    Frame* _fromFrame, *_toFrame;
  };

} // end namespace

#endif
