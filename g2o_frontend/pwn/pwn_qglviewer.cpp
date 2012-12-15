#include "pwn_qglviewer.h"
#include "pwn_math.h"
#include <GL/gl.h>
#include "g2o/stuff/opengl_primitives.h"
#include <iostream>
using namespace std;


class StandardCamera : public qglviewer::Camera{
public:
  StandardCamera() : _standard(true) {};
  
  float zNear() const{
    if (_standard) 
      return 0.001f; 
    else 
      return Camera::zNear(); 
  }

  float zFar() const{  
    if (_standard) 
      return 10000.0f; 
    else 
      return Camera::zFar();
  }

  bool standard() const { return _standard; }
  void setStandard(bool s) { _standard = s; }

private:
  bool _standard;
};

PWNQGLViewer::PWNQGLViewer(QWidget *parent, const QGLWidget *shareWidget, Qt::WFlags flags) :
  QGLViewer(parent, shareWidget, flags){
  _normalLength = 0.05f;
  _pointSize = 1.0f;
  _ellipsoidScale = 0.05f;
  _ellipsoidCrop = 0.1f;
  _ellipsoidList = -1;
  _corrVector = 0;
}

void PWNQGLViewer::init() {
  QGLViewer::init();
  setBackgroundColor(QColor::fromRgb(51, 51, 51));

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND); 
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_FLAT);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setAxisIsDrawn();

  // don't save state
  setStateFileName(QString::null);

  // mouse bindings
  setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

  // replace camera
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  setCamera(cam);
  cam->setPosition(qglviewer::Vec(0., 0., 75.));
  cam->setUpVector(qglviewer::Vec(0., 1., 0.));
  cam->lookAt(qglviewer::Vec(0., 0., 0.));
  delete oldcam;

  // create one display list
  _ellipsoidList = glGenLists(1);

  // compile the display list, store a sphere on it.
  glNewList(_ellipsoidList, GL_COMPILE);
  g2o::opengl::drawSphere(1.0f);
  glEndList();

}

void PWNQGLViewer::draw() {
  QGLViewer::draw();
  if (! _points)
    return;
  if (_pointSize>0){
    glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
    glPointSize(_pointSize);
    glBegin(GL_POINTS);
    for (size_t i=0; i<_points->size(); i++){
      const Vector6f& p = (*_points)[i];
      if (p.tail<3>().norm()>0.){
	glNormal3f(p[3], p[4], p[5]);
      }
      glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
  }
  if (_pointSize>0){
    glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
    glPointSize(_pointSize);
    glBegin(GL_POINTS);
    for (size_t i=0; i<_points2->size(); i++){
      const Vector6f& p = (*_points2)[i];
      if (p.tail<3>().norm()>0.){
	glNormal3f(p[3], p[4], p[5]);
      }
      glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
  }
  float nsum  = 0;
  if (_normalLength>0){
    glColor4f(1.0f, 0.0f, 1.0f, 0.5f);
    glPointSize(.1);
    glBegin(GL_LINES);
    for (size_t i=0; i<_points->size(); i++){
      const Vector6f& p = (*_points)[i];
      glVertex3f(p[0], p[1], p[2]);
      nsum += p.tail<3>().squaredNorm();
      glVertex3f(p[0]+p[3]*_normalLength,
		 p[1]+p[4]*_normalLength, 
		 p[2]+p[5]*_normalLength);
    }
    glEnd();
  }
  if (_ellipsoids && _ellipsoidScale>0){
    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(_pointSize*.5);
    for (size_t i=0; i<_ellipsoids->size(); i++) {
      const SVDMatrix3f& covSVD = _ellipsoids->at(i);
      const Eigen::Vector3f& lambda = covSVD.lambda;
      const Eigen::Isometry3f& I = covSVD.isometry;
      if (covSVD.lambda.squaredNorm()==0.0f)
	continue;
      float sx = sqrt(lambda[0])*_ellipsoidScale;
      float sy = sqrt(lambda[1])*_ellipsoidScale;
      float sz = sqrt(lambda[2])*_ellipsoidScale;
      float curvature = lambda[0] / (lambda[0] + lambda[1] + lambda[2]);
      glPushMatrix();
      glMultMatrixf(I.data());
      if(curvature > 0.02f){
	glColor4f(1.0f - curvature, 0.0f, 0.0f, 0.5f);
      }
      else{
	glColor4f(0.0f, 1.0f - curvature, 0.0f, 0.5f);
	sx = 1e-03;
	sy = _ellipsoidScale;
	sz = _ellipsoidScale;
      }
      glScalef(sx, sy, sz);
      glCallList(_ellipsoidList);
      glPopMatrix();
    }
  }
  if (_corrVector) {
    glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    for (size_t i =0; i< _corrVector->size(); i++){
      const Vector6f& p1 = *(_corrVector->at(i).p1);
      const Vector6f& p2 = *(_corrVector->at(i).p2);
      glVertex3f(p1.x(), p1.y(), p1.z());
      glVertex3f(p2.x(), p2.y(), p2.z());
    }
    glEnd();
  }
}
