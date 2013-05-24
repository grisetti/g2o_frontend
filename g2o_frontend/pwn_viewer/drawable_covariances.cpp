#include "drawable_covariances.h"
#include "gl_parameter_covariances.h"
#include "pwn_qglviewer.h"

using namespace pwn;

DrawableCovariances::DrawableCovariances() : Drawable() {
  _parameter = 0;
  _covariances = 0;
  _viewer = 0;
}

DrawableCovariances::DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_, PointStatsVector *covariances_) : Drawable(transformation_){
  setParameter(parameter_);
  _covariances = covariances_;
}

bool DrawableCovariances::setParameter(GLParameter *parameter_) {
  GLParameterCovariances* covariancesParameter = (GLParameterCovariances*)parameter_;
  if (covariancesParameter == 0) {
    _parameter = 0;
    return false;
  }
  _parameter = covariancesParameter;
  return true;
}

// Drawing function of the class object.
void DrawableCovariances::draw() {
  GLParameterCovariances* covariancesParameter = dynamic_cast<GLParameterCovariances*>(_parameter);
  if (_covariances && 
      covariancesParameter && 
      covariancesParameter->isShown() && 
      covariancesParameter->ellipsoidScale() > 0.0f) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    covariancesParameter->applyGLParameter();
    float ellipsoidScale = covariancesParameter->ellipsoidScale();
    Eigen::Vector4f colorLowCurvature = covariancesParameter->colorLowCurvature();
    Eigen::Vector4f colorHighCurvature = covariancesParameter->colorHighCurvature();
    float curvatureThreshold = covariancesParameter->curvatureThreshold();
    for (size_t i = 0; i < _covariances->size(); i += covariancesParameter->step()) {
      PointStats cov = _covariances->at(i);
      Eigen::Vector3f lambda = cov.eigenValues();
      Eigen::Isometry3f I = Eigen::Isometry3f::Identity();
      I.linear() = cov.eigenVectors();
      if (cov.n() == 0 )
	continue;
      I.translation() = Eigen::Vector3f(cov.mean()[0],cov.mean()[1],cov.mean()[2]);
      float sx = sqrt(lambda[0])*ellipsoidScale;
      float sy = sqrt(lambda[1])*ellipsoidScale;
      float sz = sqrt(lambda[2])*ellipsoidScale;
      float curvature = cov.curvature();
      glPushMatrix();
      glMultMatrixf(I.data());
      
      if (curvature > curvatureThreshold) {
	glColor4f(colorHighCurvature[0] - curvature, colorHighCurvature[1], colorHighCurvature[2], colorHighCurvature[3]);
	sx = ellipsoidScale;
	sy = ellipsoidScale;
	sz = ellipsoidScale;
      }
      else {
	glColor4f(colorLowCurvature[0], colorLowCurvature[1] - curvature, colorLowCurvature[2], colorLowCurvature[3]);
	sx = 1e-03;
	sy = ellipsoidScale;
	sz = ellipsoidScale;
      }
      glScalef(sx, sy, sz);
      glCallList(_viewer->ellipsoidDrawList());
      glPopMatrix();
    }
    glPopMatrix();
  }
}
