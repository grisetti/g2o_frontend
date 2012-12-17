#include "drawable_points.h"
#include "gl_parameter_points.h"

DrawablePoints::DrawablePoints() {
  _points = 0;
  GLParameterPoints* pointsParameter = new GLParameterPoints();
  _parameter = (GLParameter*)pointsParameter;
}

DrawablePoints::DrawablePoints(Vector6fVector *points_, GLParameter *parameter_) {
  _points = points_;
  setParameter(parameter_);
}

bool DrawablePoints::setParameter(GLParameter *parameter_) {
  GLParameterPoints* pointsParameter = dynamic_cast<GLParameterPoints*>(parameter_);
  if (pointsParameter == 0) {
    _parameter = 0;
    return false;
  }
  _parameter = parameter_;
  return true;
}

// Drawing function of the class object.
void DrawablePoints::draw() {
  if (_points) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    _parameter->applyGLParameter();
    glBegin(GL_POINTS);
    for (size_t i = 0; i < _points->size(); i++) {
      const Vector6f &p = (*_points)[i];
      if (p.tail<3>().norm() > 0.0f) 
	glNormal3f(p[3], p[4], p[5]);
      glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
    glPopMatrix();
  }
}
