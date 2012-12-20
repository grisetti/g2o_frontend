#ifndef GL_PARAMETER
#define GL_PARAMETER

#include <GL/gl.h>

class GLParameter {
 public:
  GLParameter();
  virtual void applyGLParameter() = 0;
};

#endif
