#include "drawable.h"

Drawable::Drawable() {
  _transformation = Eigen::Isometry3f::Identity();
  _parameter = 0;
  _dmQGLViewer = 0;
  _step = 1;
}
