#ifndef _BOSS_FRAME_RELATION_H_
#define _BOSS_FRAME_RELATION_H_

#include "reference_frame.h"

namespace boss_map {
  using namespace boss;
  //! a frame relation is a transform (eventually with covariance), that specifies the relative transformation
  //! between two frames
  class ReferenceFrameRelation: public boss::Identifiable{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ReferenceFrameRelation(int id=-1, boss::IdContext* context = 0);
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserializeComplete();

    inline const Eigen::Isometry3d& transform() {return _transform;}
    inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
  
    inline const Eigen::Matrix<double, 6,6>& informationMatrix() {return _informationMatrix;}
    inline void setInformationMatrix(const Eigen::Matrix<double, 6,6>& informationMatrix_) {_informationMatrix=informationMatrix_;}

    inline const ReferenceFrame* fromReferenceFrame() const { return _fromReferenceFrame;}
    inline ReferenceFrame* fromReferenceFrame() { return _fromReferenceFrame;}
    inline void setFromReferenceFrame(ReferenceFrame* from_) {_fromReferenceFrame = from_;}


    inline const ReferenceFrame* toReferenceFrame() const { return _toReferenceFrame;}
    inline ReferenceFrame* toReferenceFrame() { return _toReferenceFrame;}
    inline void setToReferenceFrame(ReferenceFrame* to_) {_toReferenceFrame = to_;}

  protected:
    Eigen::Isometry3d _transform;
    Eigen::Matrix<double, 6,6> _informationMatrix;
    ReferenceFrame* _fromReferenceFrame, *_toReferenceFrame;
  };

} // end namespace

#endif
