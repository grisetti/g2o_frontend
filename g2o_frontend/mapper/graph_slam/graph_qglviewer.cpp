#include "graph_qglviewer.h"
#include "../../sensor_data/laser_robot_data.h"

#include <fstream>
#include <GL/gl.h>

#define RAG2DEG(x) (x*180/M_PI)


using namespace g2o;



GraphQGLViewer::GraphQGLViewer(QWidget *parent): QGLViewer(parent)
{
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
}


void GraphQGLViewer::drawVertex()
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


void GraphQGLViewer::drawReferenceGraph()
{
    _drawableVertices.clear();
    _drawableEdges.clear();

    setAlpha(1.0f);
    setColor(0, 0, 1);
    setDepth(0.0f);
    drawGraph(_referenceGraphVertices);
}


void GraphQGLViewer::drawGraph(const Vertices &v)
{
    glNormal3f(0.f, 0.f, 1.f);
    for(size_t i = 0; i < v.size(); ++i)
    {
        VertexSE2* vse2 = dynamic_cast<VertexSE2*>(v[i]);
        VertexSE3* vse3 = dynamic_cast<VertexSE3*>(v[i]);
        if(vse2 && !vse3)
        {
            OptimizableGraph::Data* d = vse2->userData();
            const LaserRobotData* laserRobotData = dynamic_cast<const LaserRobotData*>(d);
            if(laserRobotData)
            {
                glPushMatrix();
                glColor4f(_red, _green, _blue, _alpha);
                glTranslatef((float)vse2->estimate().translation().x(), (float)vse2->estimate().translation().y(), _depth);
                glRotatef((float)(RAG2DEG(vse2->estimate().rotation().angle())), 0.f, 0.f, 1.f);
                glNormal3f(0.f,0.f,1.f);
                drawVertex();
//                LaserRobotData::Vector2fVector scan = laserRobotData->floatCartesian();
//                for(size_t i = 0; i < scan.size(); ++i)
//                {
//                    glBegin(GL_POINTS);
//                    glColor4f(_red, _green, _blue, _alpha);
//                    glVertex3f(scan[i].x(), scan[i].y(), 0.f);
//                    glEnd();
//                }
                glPopMatrix();
            }
        }
        if(vse3 && !vse2)
        {
            glPushMatrix();
            glColor4f(_red, _green, _blue, _alpha);
            glMultMatrixd(vse3->estimate().matrix().data());
            glNormal3f(0.f,0.f,1.f);
            drawVertex();
            glPopMatrix();
        }
    }
    glColor3f(1,1,1);
}


void GraphQGLViewer::draw()
{
    drawAxis();
    drawReferenceGraph();
}
