#include "drawable_points.h"

namespace pwn_viewer {

  DrawablePoints::DrawablePoints() : Drawable() {
    _parameter = 0;
    _points = 0;
    _normals = 0;
    _traversabilityVector = 0;    
    _pointDrawList = glGenLists(1);
    updatePointDrawList();
  }

  DrawablePoints::DrawablePoints(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, PointVector *points_,  
				 NormalVector *normals_) : Drawable(transformation_) {
    setParameter(parameter_);
    _points = points_;
    _normals = normals_;
    _traversabilityVector = 0;
    _pointDrawList = glGenLists(1);
    updatePointDrawList();
  }

  bool DrawablePoints::setParameter(GLParameter *parameter_) {
    GLParameterPoints *pointsParameter = (GLParameterPoints*)parameter_;
    if(pointsParameter == 0) {
      _parameter = 0;
      return false;
    }
    _parameter = pointsParameter;
    return true;
  }

  void DrawablePoints::draw() {
    GLParameterPoints *pointsParameter = dynamic_cast<GLParameterPoints*>(_parameter);
    if(_pointDrawList &&
       pointsParameter && 
       pointsParameter->show() && 
       pointsParameter->pointSize() > 0.0f) {
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      glCallList(_pointDrawList);
      glPopMatrix();
    }
  }

  void DrawablePoints::updatePointDrawList() {
    GLParameterPoints *pointsParameter = dynamic_cast<GLParameterPoints*>(_parameter);
    glNewList(_pointDrawList, GL_COMPILE);    
    if(_pointDrawList &&
       _points && 
       _normals &&
       pointsParameter && 
       pointsParameter->pointSize() > 0.0f) {
      pointsParameter->applyGLParameter();
      glBegin(GL_POINTS);
      for(size_t i = 0; i < _points->size(); i += pointsParameter->step()) {
      	const Point &p = _points->at(i);
      	const Normal &n = _normals->at(i);
      	glNormal3f(n[0], n[1], n[2]);
      	glVertex3f(p[0], p[1], p[2]);
      }    
      glEnd();
    }
    glEndList();
  }

}
