#include "drawable_points.h"
#include "gl_parameter_points.h"

DrawablePoints::DrawablePoints() : Drawable() {
  GLParameterPoints* pointsParameter = new GLParameterPoints();
  _parameter = (GLParameter*)pointsParameter;
  _points = 0;
  _normals = 0;
}

DrawablePoints::DrawablePoints(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, PointVector *points_,  NormalVector *normals_) : Drawable(transformation_) {
  setParameter(parameter_);
  _points = points_;
  _normals = normals_;
}

bool DrawablePoints::setParameter(GLParameter *parameter_) {
  GLParameterPoints* pointsParameter = (GLParameterPoints*)parameter_;
  if (pointsParameter == 0) {
    _parameter = 0;
    return false;
  }
  _parameter = parameter_;
  return true;
}

// Drawing function of the class object.
void DrawablePoints::draw() {
  GLParameterPoints *pointsParameter = dynamic_cast<GLParameterPoints*>(_parameter);
  
  if (_points && 
      _normals && 
      pointsParameter && 
      pointsParameter->isShown() && 
      pointsParameter->pointSize() > 0.0f) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    pointsParameter->applyGLParameter();
    glBegin(GL_POINTS);
    for (size_t i = 0; i < _points->size(); i += pointsParameter->step()) {
      const Point &p = _points->at(i);
      const Normal &n = _normals->at(i);
      glNormal3f(n[0], n[1], n[2]);
      glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
    glPopMatrix();
  }
}
