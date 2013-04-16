#ifndef _PIXEL_MAPPER_H_
#define _PIXEL_MAPPER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

class PixelMapper{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PixelMapper();

  inline const Eigen::Isometry3f& transform() const {return _transform;}

  inline void setTransform(const Eigen::Isometry3f& transform_) {
    _transform = transform_;
    updateTransform();
  }

  inline const Eigen::Matrix3f& cameraMatrix() const {return _cameraMatrix;}
  
  inline void setCameraMatrix(const Eigen::Matrix3f& cameraMatrix_) {
    _cameraMatrix = cameraMatrix_;
    updateTransform();
  } 

  /*3f interface*/
  inline Eigen::Vector2i imageCoords(const Eigen::Vector3f& projected) const  {
    return Eigen::Vector2i(projected(0)/projected(2), projected(1)/projected(2));
  }

  inline Eigen::Vector3f projectPoint(const Eigen::Vector3f& v) const {
    return _KR * v + _Kt;
  }

  inline Eigen::Vector3f unprojectPixel(int x, int y, float depth) const {
    Eigen::Vector3f ip(depth*x, depth*y, depth);
    return _iKR*ip+_iKt;
  }

  /*4f interface*/
  inline Eigen::Vector2i imageCoords4f(const Eigen::Vector4f& projected) const  {
    return Eigen::Vector2i(projected(0)/projected(2), projected(1)/projected(2));
  }

  inline Eigen::Vector4f projectPoint4f(const Eigen::Vector4f& v) const {
    return _KRt * v;
  }

  inline Eigen::Vector4f unprojectPixel4f(int x, int y, float depth) const {
    Eigen::Vector4f ip(depth*x, depth*y, depth, 1.0);
    return _iKRt*ip;
  }

protected:
  void updateTransform();
  Eigen::Matrix3f _KR;
  Eigen::Vector3f _Kt;
  Eigen::Matrix3f _iKR;
  Eigen::Vector3f _iKt;
  Eigen::Matrix4f _KRt;
  Eigen::Matrix4f _iKRt;

  Eigen::Isometry3f _transform;
  Eigen::Matrix3f _cameraMatrix;
};


#endif
