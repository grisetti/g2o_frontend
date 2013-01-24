#include "drawable_points.h"
#include "gl_parameter_points.h"
#include <iostream>

using namespace std;

DrawablePoints::DrawablePoints() : Drawable() {
  GLParameterPoints* pointsParameter = new GLParameterPoints();
  _parameter = (GLParameter*)pointsParameter;
  _points = 0;
}

DrawablePoints::DrawablePoints(Eigen::Isometry3f transformation_, GLParameter *parameter_,  PointWithNormalVector *points_) : Drawable(transformation_) {
  setParameter(parameter_);
  _points = points_;
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
  if (_points && pointsParameter && pointsParameter->pointSize() > 0.0f) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    pointsParameter->applyGLParameter();
    glBegin(GL_POINTS);
    for (size_t i = 0; i < _points->size(); i += pointsParameter->step()) {
      const PointWithNormal& p = (*_points)[i];
      if (p.tail<3>().squaredNorm() > 0.0f) {
	glNormal3f(p[3], p[4], p[5]);
      }
      glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
    glPopMatrix();
  }
}
