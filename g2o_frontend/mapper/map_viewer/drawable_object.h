#ifndef DRAWABLE_OBJECT_H
#define DRAWABLE_OBJECT_H


#include "QGLViewer/frame.h"



class DrawableObject
{
public:
    DrawableObject();
    virtual ~DrawableObject();
    virtual void draw();

    qglviewer::Frame frame;
};
#endif //DRAWABLE_OBJECT_H
