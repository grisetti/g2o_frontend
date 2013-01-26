#ifndef GL_PARAMETER_CORRESPONDENCES
#define GL_PARAMETER_CORRESPONDENCES

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

class GLParameterCorrespondences : public GLParameter{
 public:
  GLParameterCorrespondences();
  GLParameterCorrespondences(float pointSize_, Eigen::Vector4f color_, float lineWidth_);
  void setPointSize(float pointSize_) { _pointSize = pointSize_; }
  void setColor(Eigen::Vector4f color_) { _color = color_; }
  void setLineWidth(float lineWidth_) { _lineWidth = lineWidth_; }
  float pointSize() { return _pointSize; }
  Eigen::Vector4f color() { return _color; }
  float lineWidth() { return _lineWidth; }
  virtual void applyGLParameter();

 protected:
  float _pointSize;
  Eigen::Vector4f _color;
  float _lineWidth;
};

#endif
