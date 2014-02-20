#ifndef DRAWABLE_VERTEX_H
#define DRAWABLE_VERTEX_H


#include "drawable_object.h"



class DrawableVertex : public DrawableObject
{
public:
    DrawableVertex();
    virtual void draw();
protected:
    int _graphId;
};
#endif //DRAWABLE_VERTEX_H
