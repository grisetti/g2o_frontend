#ifndef GL_PARAMETER
#define GL_PARAMETER

#include <GL/gl.h>

class GLParameter {
 public:
  GLParameter();
  virtual void applyGLParameter() = 0;
  void setStep(int step_) { _step = step_; }
  int step() { return _step; }
protected:
  int _step;
};

#endif
