#include "pixelmapper.h"
PixelMapper::PixelMapper(){
  _cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  _transform.setIdentity();
  updateTransform();
}

void PixelMapper::updateTransform(){
  Eigen::Isometry3f t=_transform.inverse();
  Eigen::Matrix3f iK=_cameraMatrix.inverse();
  _KR = _cameraMatrix*t.linear();
  _Kt = _cameraMatrix*t.translation();
  _iKR = _transform.linear()*iK;
  _iKt = _transform.translation();
}
