#ifndef DRAWABLE
#define DRAWABLE

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

class Drawable {
 public:
  Drawable();
  Drawable(Eigen::Isometry3f transformation_, GLParameter *parameter_);
  virtual void draw() {}
  void setTransformation(Eigen::Isometry3f transformation_) { _transformation = transformation_; }
  virtual bool setParameter(GLParameter *parameter_) = 0; //{ _parameter = parameter_; }
  Eigen::Isometry3f transformation() { return _transformation; }
  virtual GLParameter* parameter() = 0;//{ return _parameter; }

 protected:
  // Isometry transformation to apply when draw method is called.
  Eigen::Isometry3f _transformation;
  // Drawing p0arameter.
  GLParameter *_parameter;
};

#endif
