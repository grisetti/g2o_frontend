#include "pwn_qglviewer.h"
#include <GL/gl.h>

PWNQGLViewer::PWNQGLViewer(QWidget *parent, const QGLWidget *shareWidget, Qt::WFlags flags) :
  QGLViewer(parent, shareWidget, flags){
  _normalLength = 0.;
  _pointSize = 0.;
  _ellipsoidsScale = 0;
}

void PWNQGLViewer::init() {
  glDisable(GL_LIGHTING);
}

void PWNQGLViewer::draw() {
  if (! _points)
    return;
  if (_pointSize>0){
    glColor3f(0.5,0.5,0.5);
    glPointSize(_pointSize);
    glBegin(GL_POINTS);
    for (size_t i=0; i<_points->size(); i++){
      const Vector6f& p = (*_points)[i];
      if (p.tail<3>().norm()>0.){
	glNormal3f(p[3], p[4], p[5]);
      }
      glVertex3f(p[0],p[1], p[2]);
    }
    glEnd();
  }
  if (_normalLength>0){
    glColor3f(0.3,0.3,0.3);
    glPointSize(_pointSize*.5);
    glBegin(GL_LINES);
    for (size_t i=0; i<_points->size(); i++){
      const Vector6f& p = (*_points)[i];
      glVertex3f(p[0],p[1], p[2]);
      glVertex3f(p[0]+p[3]*_normalLength,
		 p[1]+p[4]*_normalLength, 
		 p[2]+p[5]*_normalLength);
    }
    glEnd();
  }
  if (_ellipsoids && _ellipsoidsScale>0){
  }
}
