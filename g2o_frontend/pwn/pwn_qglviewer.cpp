#include "pwn_qglviewer.h"
#include "pwn_math.h"
#include <GL/gl.h>

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
  _normalLength = 0.0f;
  _pointSize = 1.0f;
  _ellipsoidsScale = 0.05f;
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
}

void PWNQGLViewer::draw() {
  QGLViewer::draw();
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
  float nsum  = 0;
  if (_normalLength>0){
    glColor3f(0.3,0.3,0.3);
    glPointSize(_pointSize*.5);
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
  if (_ellipsoids && _ellipsoidsScale>0){
    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(_pointSize*.5);
    glBegin(GL_LINES);
    for (int i=0; i<_ellipsoids->rows(); i++){
      for (int j=0; j<_ellipsoids->cols(); j++){
	covarianceSVD covSVD = (*_ellipsoids)(i, j);
	Eigen::Vector3f lambda = covSVD.lambda;
	Eigen::Isometry3f I = covSVD.isometry;
	if(covSVD.lambda == Eigen::Vector3f::Zero())
	  continue;
	Eigen::Vector3f t = I.translation();
	Eigen::Matrix3f R = I.linear();
	Eigen::Vector3f v = mat2quat(R.transpose());
	glPushMatrix();
	glTranslatef(t[0], t[1], t[2]);
	/*glRotatef(v[0]*180.0f/3.14f, 1.0f, 0.0f, 0.0f);
	glRotatef(v[1]*180.0f/3.14f, 0.0f, 1.0f, 0.0f);
	glRotatef(v[2]*180.0f/3.14f, 0.0f, 0.0f, 1.0f);*/
	glBegin(GL_LINES);
	glVertex3f(-lambda[0]*_ellipsoidsScale, 0.0f, 0.0f);
	glVertex3f(lambda[0]*_ellipsoidsScale, 0.0f, 0.0f);
	glVertex3f(0.0f, -lambda[1]*_ellipsoidsScale, 0.0f);
	glVertex3f(0.0f, lambda[1]*_ellipsoidsScale, 0.0f);
	glVertex3f(0.0f, 0.0f, -lambda[2]*_ellipsoidsScale);
	glVertex3f(0.0f, 0.0f, lambda[2]*_ellipsoidsScale);
	glEnd();
	glPopMatrix();
      }
    }
  }
}
