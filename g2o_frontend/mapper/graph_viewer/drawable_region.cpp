#include "drawable_region.h"
#include "graph_qglviewer.h"


using namespace g2o;
using namespace std;



DrawableRegion::DrawableRegion() : Drawable()
{
    _region = 0;
    _drawableRegion = new list<DrawableVertex*>;
}


DrawableRegion::DrawableRegion(HyperGraph::VertexSet* region_) : Drawable()
{
    _region = region_;
    _drawableRegion = new list<DrawableVertex*>;
}


void DrawableRegion::setRegion(HyperGraph::VertexSet* region_)
{
    _region = region_;
    for(HyperGraph::VertexSet::iterator it = _region->begin(); it != _region->end(); ++it)
    {
        HyperGraph::Vertex* v = *it;
        VertexSE2* vse2 = dynamic_cast<VertexSE2*>(v);
        DrawableVertex* dv = new DrawableVertex;
        dv->setVertex(vse2);
        _drawableRegion->push_back(dv);
    }
}

void DrawableRegion::draw()
{
    glColor4f(.0f, 1.0f, .0f, 1.f);
    for(list<DrawableVertex*>::const_iterator it = _drawableRegion->begin(); it != _drawableRegion->end(); ++it)
    {
        DrawableVertex* v = new DrawableVertex;
        v->setVertex((*it)->vertex());
        v->draw();
    }
}
