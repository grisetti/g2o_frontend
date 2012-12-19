#include "drawable.h"

Drawable::Drawable() {
  _transformation = Eigen::Isometry3f::Identity();
  //_parameter = 0;
  _dmQGLViewer = 0;
  _step = 1;
}

Drawable::Drawable(Eigen::Isometry3f transformation_, int step_) {
  _transformation = transformation_;
  _dmQGLViewer = 0;
  _step = step_;
}
