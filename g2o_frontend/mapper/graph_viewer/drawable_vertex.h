#ifndef DRAWABLE_VERTEX_H
#define DRAWABLE_VERTEX_H


#include "drawable.h"
#include "g2o/types/slam2d/vertex_se2.h"



class DrawableVertex : public Drawable
{
public:
    DrawableVertex();
    DrawableVertex(g2o::VertexSE2* vertex_);

    inline virtual void setVertex(g2o::VertexSE2* vertex_) { _vertex = vertex_; }
    inline virtual void setPayload(Drawable* d_) { _payload = d_; }

    inline virtual g2o::VertexSE2* vertex() { return _vertex; }
    inline virtual const g2o::VertexSE2* vertex() const { return _vertex; }
    inline virtual Drawable* payload() {return _payload; }
    inline virtual const Drawable* payload() const {return _payload; }

    virtual void draw();

protected:
    g2o::VertexSE2* _vertex;
    Drawable* _payload;

};
#endif //DRAWABLE_VERTEX_H
