#ifndef GL_PARAMETER_POINTS
#define GL_PARAMETER_POINTS

#include <Eigen/Geometry>
#include <GL/gl.h>

class GLParameterPoints {
 public:
  GLParameterPoints();
  GLParameterPoints(float pointSize_, Eigen::Vector4f color_);
  void setPointSize(float pointSize_) { _pointSize = pointSize_; }
  void setColor(Eigen::Vector4f color_) { _color = color_; }
  float pointSize() { return _pointSize; }
  Eigen::Vector4f color() { return _color; }
  virtual void applyGLParameter();

 protected:
  float _pointSize;
  Eigen::Vector4f _color;
};

#endif
