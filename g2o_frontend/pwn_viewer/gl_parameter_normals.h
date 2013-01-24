#ifndef GL_PARAMETER_NORMALS
#define GL_PARAMETER_NORMALS

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

class GLParameterNormals: public GLParameter {
 public:
  GLParameterNormals();
  GLParameterNormals(float pointSize_, Eigen::Vector4f color_, float normalLength_);
  void setPointSize(float pointSize_) { _pointSize = pointSize_; }
  void setColor(Eigen::Vector4f color_) { _color = color_; }
  void setNormalLength(float normalLength_) { _normalLength = normalLength_; }
  float pointSize() { return _pointSize; }
  Eigen::Vector4f color() { return _color; }
  float normalLength() { return _normalLength; }
  virtual void applyGLParameter();

 protected:
  float _pointSize;
  Eigen::Vector4f _color;
  float _normalLength;
};

#endif
