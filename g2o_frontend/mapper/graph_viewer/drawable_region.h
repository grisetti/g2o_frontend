#ifndef DRAWABLE_REGION_H
#define DRAWABLE_REGION_H


#include "drawable_vertex.h"
#include <list>


class DrawableRegion : public Drawable
{
public:
    DrawableRegion();
    DrawableRegion(g2o::HyperGraph::VertexSet* region_);

    void setRegion(g2o::HyperGraph::VertexSet* region_);

    inline virtual g2o::HyperGraph::VertexSet* region() {return _region; }
    inline virtual const g2o::HyperGraph::VertexSet* region() const {return _region; }

    virtual void draw();

protected:
    g2o::HyperGraph::VertexSet* _region;
    std::list<DrawableVertex*>* _drawableRegion;

};
#endif // DRAWABLE_REGION_H
