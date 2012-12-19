#ifndef DRAWABLE
#define DRAWABLE

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

class DMQGLViewer;

class Drawable {
 public:
  Drawable();
  Drawable(Eigen::Isometry3f transformation_, int step_);
  virtual void draw() {}
  void setTransformation(Eigen::Isometry3f transformation_) { _transformation = transformation_; }
  virtual bool setParameter(GLParameter *parameter_) = 0;
  void setDMQGLViewer(DMQGLViewer *dmQGLViewer_) { _dmQGLViewer = dmQGLViewer_; }
  void setStep(int step_) { _step = step_; }
  DMQGLViewer* dmQGLViewer() { return _dmQGLViewer; }
  int step() { return _step; }
  Eigen::Isometry3f transformation() { return _transformation; }
  virtual GLParameter* parameter() = 0;

 protected:
  // Isometry transformation to apply when draw method is called.
  Eigen::Isometry3f _transformation;
  // Drawing p0arameter.
  GLParameter *_parameter;
  // Pointer to the DMQGLViewer.
  DMQGLViewer *_dmQGLViewer;
  // Draw stuffs reduction.
  int _step;
};

#endif
