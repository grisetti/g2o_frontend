#include <iostream>
#include <GL/gl.h>
#include "pwn_qglviewer.h"
#include "g2o/stuff/opengl_primitives.h"

using namespace std;

class StandardCamera : public qglviewer::Camera {
public:
  StandardCamera() : _standard(true) {};
  
  float zNear() const {
    if (_standard) 
      return 0.001f; 
    else 
      return Camera::zNear(); 
  }

  float zFar() const {  
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
  _ellipsoidDrawList = 0;
  _numDrawLists = 2;
}

// Initialization function for the viewer.
void PWNQGLViewer::init() {
  // Init QGLViewer.
  QGLViewer::init();
  // Set background color light yellow.
  setBackgroundColor(QColor::fromRgb(255, 255, 194));

  // Set some default settings.
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND); 
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_FLAT);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Don't save state.
  setStateFileName(QString::null);

  // Mouse bindings.
  setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

  // Replace camera.
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  setCamera(cam);
  cam->setPosition(qglviewer::Vec(0.0f, 0.0f, -1.0f));
  cam->setUpVector(qglviewer::Vec(0.0f, -1.0f, 0.0f));
  cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));
  delete oldcam;

  // Create draw lists.
  _ellipsoidDrawList = glGenLists(_numDrawLists);
  _pyramidDrawList = glGenLists(_numDrawLists);

  // Compile draw lists.
  // Ellipsoid.
  glNewList(_ellipsoidDrawList, GL_COMPILE);
  g2o::opengl::drawSphere(1.0f);
  glEndList();
  // Pyramid.
  glNewList(_pyramidDrawList, GL_COMPILE);
  g2o::opengl::drawPyramid(0.5f, 0.5f);
  glEndList();
}

// Function containing the draw commands.
void PWNQGLViewer::draw() {
  QGLViewer::draw();
  
  // Draw camera object.
  glPushMatrix();
  glColor4f(1.0f, 0.0f, 1.0f, 0.5f);
  glScalef(0.05f, 0.05f, 0.1f);
  glCallList(_pyramidDrawList);
  glPopMatrix();

  // Draw the vector of drawable objects.
  for (size_t i = 0; i < _drawableList.size(); i++) {
    _drawableList[i]->draw();
  }
}

// Function to add a drawable objects to the viewer.
void PWNQGLViewer::addDrawable(Drawable *d) {
  // Set the viewer to the input drawable object.
  d->setViewer(this);
  // Add the input object to the vector.
  _drawableList.push_back(d);
}
