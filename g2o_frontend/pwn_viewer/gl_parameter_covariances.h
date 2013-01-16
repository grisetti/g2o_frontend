#ifndef GL_PARAMETER_COVARIANCES
#define GL_PARAMETER_COVARIANCES

#include <Eigen/Geometry>
#include <GL/gl.h>

class GLParameterCovariances {
 public:
  GLParameterCovariances();
  GLParameterCovariances(float pointSize_, Eigen::Vector4f colorLowCurvature_, Eigen::Vector4f colorHighCurvature_, float curvatureThreshold_, float ellipsoidScale_);
  void setPointSize(float pointSize_) { _pointSize = pointSize_; }
  void setColorLowCurvature(Eigen::Vector4f colorLowCurvature_) { _colorLowCurvature = colorLowCurvature_; }
  void setColorHighCurvature(Eigen::Vector4f colorHighCurvature_) { _colorHighCurvature = colorHighCurvature_; }
  void setCurvatureThreshold(float curvatureThreshold_) { _curvatureThreshold = curvatureThreshold_; }
  void setEllipsoidScale(float ellipsoidScale_) { _ellipsoidScale = ellipsoidScale_; }
  float pointSize() { return _pointSize; }
  Eigen::Vector4f colorLowCurvature() { return _colorLowCurvature; }
  Eigen::Vector4f colorHighCurvature() { return _colorHighCurvature; }
  float curvatureThreshold() { return _curvatureThreshold; }
  float ellipsoidScale() { return _ellipsoidScale; }
  virtual void applyGLParameter();

 protected:
  float _pointSize;
  Eigen::Vector4f _colorLowCurvature;
  Eigen::Vector4f _colorHighCurvature;
  float _curvatureThreshold;
  float _ellipsoidScale;
};

#endif
