#ifndef DRAWABLE
#define DRAWABLE

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn {

class PWNQGLViewer;

class Drawable {
 public:
  Drawable();
  Drawable(Eigen::Isometry3f transformation_);
  virtual ~Drawable() {}
  
  virtual void draw() {}

  Eigen::Isometry3f transformation() { return _transformation; }
  PWNQGLViewer* viewer() { return _viewer; }
  virtual GLParameter* parameter() = 0;
  
  virtual void setTransformation(Eigen::Isometry3f transformation_) { _transformation = transformation_; }
  virtual void setViewer(PWNQGLViewer *viewer_) { _viewer = viewer_; }
  virtual bool setParameter(GLParameter *parameter_) = 0;

  //virtual void updatePointDrawList() { }

 protected:
  Eigen::Isometry3f _transformation;
  PWNQGLViewer *_viewer;
};

}

#endif
