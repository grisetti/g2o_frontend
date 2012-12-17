#include "drawable.h"

Drawable::Drawable() {
  _transformation = Eigen::Isometry3f::Identity();
  _parameter = 0;
}

Drawable::Drawable(Eigen::Isometry3f transformation_, GLParameter *parameter_) {
  _transformation = transformation_;
  _parameter = parameter_;
}
