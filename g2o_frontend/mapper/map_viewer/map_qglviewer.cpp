#include "map_qglviewer.h"
#include <fstream>
#include <GL/gl.h>


#define RAG2DEG(x) (x*180/M_PI)

using namespace g2o;



MapQGLViewer::MapQGLViewer(QWidget *parent): QGLViewer(parent)
{
	setAxisIsDrawn(false);
} 

void MapQGLViewer::init()
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
}


void MapQGLViewer::drawVertex()
{
    glBegin(GL_LINES);
    glVertex2f(0.f, 0.f);
    glVertex2f(0.25f, 0.f);
    glEnd();

    glNormal3f(0.f,0.f,1.f);
    glBegin(GL_TRIANGLES);
    glVertex2f(0.25f, 0.f);
    glVertex2f(0.25f - 0.06f,  0.5f*0.05f);
    glVertex2f(0.25f - 0.06f, -0.5f*0.05f);
    glEnd();
}


void MapQGLViewer::draw()
{
    VertexSE2* v = new VertexSE2;
    EdgeSE2* e = new EdgeSE2;

    drawAxis();
    glNormal3f(0.f, 0.f, 1.f);
    for(size_t i = 0; i < _drawableVertices.size(); ++i)
    {
        v = _drawableVertices[i];

        glPushMatrix();
        glColor4f(1.f, .5f, .0f, .5f);
        glTranslatef((float)v->estimate().translation().x(), (float)v->estimate().translation().y(), 0.f);
        glRotatef((float)(RAG2DEG(v->estimate().rotation().angle())), 0.f, 0.f, 1.f);
        glNormal3f(0.f,0.f,1.f);
        drawVertex();
        glPopMatrix();
    }

    for(size_t j = 0; j < _drawableEdges.size(); ++j)
    {
        e = _drawableEdges[j];
        VertexSE2* from = static_cast<VertexSE2*>(e->vertex(0));
        VertexSE2* to = static_cast<VertexSE2*>(e->vertex(1));

        glBegin(GL_LINES);
        glColor4f(.5f, .5f, .5f, .5f);
        glVertex3f(from->estimate().translation().x(), from->estimate().translation().y(), 0.f);
        glVertex3f(to->estimate().translation().x(), to->estimate().translation().y(), 0.f);
        glEnd();
    }
    glColor3f(1,1,1);
}


void MapQGLViewer::setDataPointer(const Vertices drawableVertices_, const Edges drawableEdges_)
{
    _drawableVertices = drawableVertices_;
    _drawableEdges = drawableEdges_;
}
