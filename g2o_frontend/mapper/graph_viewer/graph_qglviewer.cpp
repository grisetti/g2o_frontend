#include "graph_qglviewer.h"
#include "g2o/stuff/opengl_primitives.h"
#include <GL/gl.h>

#define RAG2DEG(x) (x*180/M_PI)


using namespace g2o;



GraphQGLViewer::GraphQGLViewer(QWidget *parent) : QGLViewer(parent)
{
    _arrowDrawList = 0;
    _numDrawLists = 2;
}


void GraphQGLViewer::init()
{
    QGLViewer::init();
    //	Light disabled
    // 	glDisable(GL_LIGHTING);
    // 	glDisable(GL_LIGHT0);
    // 	glDisable(GL_LIGHT1);

    setBackgroundColor(QColor::fromRgb(51, 51, 51));

    // some default settings i like
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    //    glEnable(GL_CULL_FACE);
    glShadeModel(GL_FLAT);
    //    glShadeModel(GL_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE );

    setAxisIsDrawn(false);

    // don't save state
    setStateFileName(QString::null);

    // mouse bindings
    setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::MiddleButton, CAMERA, TRANSLATE);

    // keyboard shortcuts
    setShortcut(CAMERA_MODE, 0);
    setShortcut(EXIT_VIEWER, 0);
    //    setShortcut(SAVE_SCREENSHOT, 0);

    // replace camera
    qglviewer::Camera* oldcam = camera();
    qglviewer::Camera* cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0., 0., 75.));
    cam->setUpVector(qglviewer::Vec(0., 1., 0.));
    cam->lookAt(qglviewer::Vec(0., 0., 0.));
    delete oldcam;

    // Create draw lists.
    _arrowDrawList = glGenLists(_numDrawLists);

    // Compile draw lists.
    glNewList(_arrowDrawList, GL_COMPILE);
    g2o::opengl::drawArrow2D(.2f, .05f, .06f);
    glEndList();
}


void GraphQGLViewer::draw()
{
    QGLViewer::draw();

    for(size_t i = 0; i < _drawableList.size(); ++i)
    {
        glPushMatrix();
        _drawableList[i]->draw();
        glPopMatrix();
    }
}


void GraphQGLViewer::addDrawable(Drawable* d)
{
    d->setViewer(this);
    _drawableList.push_back(d);
}
