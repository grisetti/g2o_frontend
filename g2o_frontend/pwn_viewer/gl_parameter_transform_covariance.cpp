#include "gl_parameter_transform_covariance.h"

namespace pwn {

  GLParameterTransformCovariances::GLParameterTransformCovariances() : GLParameter() {
    _color = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
    _scale = 0.05f;
  }

  GLParameterTransformCovariances::GLParameterTransformCovariances(Eigen::Vector4f color_, float scale_) : GLParameter() {
    _color = color_;
    _scale = scale_;
  }

  void GLParameterTransformCovariances::applyGLParameter() {}

}
