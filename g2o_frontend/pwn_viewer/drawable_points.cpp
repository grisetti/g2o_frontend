#include "drawable_points.h"
#include "gl_parameter_points.h"
#include <iostream>

using namespace std;

DrawablePoints::DrawablePoints() : Drawable() {
  GLParameterPoints* pointsParameter = new GLParameterPoints();
  _parameter = (GLParameter*)pointsParameter;
}

DrawablePoints::DrawablePoints(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, 
			       const HomogeneousPoint3fVector &points_, const HomogeneousNormal3fVector &normals_) : Drawable(transformation_) {
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
  if (_points.size() > 0 && pointsParameter && pointsParameter->isShown() && pointsParameter->pointSize() > 0.0f) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    pointsParameter->applyGLParameter();
    glBegin(GL_POINTS);
    
    for (size_t i = 0; i < _points.size(); i += pointsParameter->step()) {
      const HomogeneousPoint3f p = _points[i];
      const HomogeneousNormal3f n = _normals[i];
      glNormal3f(n.x(), n.y(), n.z());
      glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();
    glPopMatrix();
  }
}
