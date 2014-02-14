#include "viewer.h"
#include "drawable.h"


using namespace std;


StandardCamera::StandardCamera() : _standard(true) {}

float StandardCamera::zNear() const
{
    if(_standard)
    {
        return 0.001f;
    }
    else
    {
        return Camera::zNear();
    }
}

float StandardCamera::zFar() const
{
    if(_standard)
    {
        return 10000.0f;
    }
    else
    {
        return Camera::zFar();
    }
}


GraphViewer::GraphViewer(QWidget *parent, const QGLWidget *shareWidget, Qt::WFlags flags): QGLViewer(parent,shareWidget,flags){}


GraphViewer::~GraphViewer() {}


void GraphViewer::init()
{
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
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 10.0f));
    cam->setUpVector(qglviewer::Vec(1.0f, 0.0f, 0.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    delete oldcam;
}


void GraphViewer::draw()
{
    for(set<DrawableItem*>::iterator it = _items.begin(); it != _items.end(); it++)
    {
        DrawableItem* item = *it;
        item->draw();
    }
}
