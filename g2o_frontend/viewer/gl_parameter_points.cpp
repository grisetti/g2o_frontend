#include "gl_parameter_points.h"

GLParameterPoints::GLParameterPoints() {
  _pointSize = 1.0f;
  _color = Eigen::Vector4f(1.0f, 1.0f, 0.0f, 0.5f);
}

GLParameterPoints::GLParameterPoints(float pointSize_, Eigen::Vector4f color_) {
  _pointSize = pointSize_;
  color_ = color_;
}

void GLParameterPoints::applyGLParameter() {
  glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
  glPointSize(1.0f);
}
