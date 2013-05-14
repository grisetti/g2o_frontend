#include "gl_parameter_points.h"

using namespace std;
GLParameterPoints::GLParameterPoints() {
  _pointSize = 1.0f;
  _color = Eigen::Vector4f(1.0f, 1.0f, 0.0f, 0.5f);
}

GLParameterPoints::GLParameterPoints(float pointSize_, const Eigen::Vector4f& color_) {
  _pointSize = pointSize_;
  _color = color_;
}

void GLParameterPoints::applyGLParameter() {
  glColor4f(_color[0], _color[1], _color[2], _color[3]);
  glPointSize(_pointSize);
}
