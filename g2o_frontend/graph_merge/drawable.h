#ifndef DRAWABLE_H
#define DRAWABLE_H

#include "drawable.h"


class Graph;
class GraphViewer;



struct DrawableItem
{
    DrawableItem(GraphViewer* viewer);
    virtual ~DrawableItem();

    virtual void draw() = 0;

    GraphViewer* _viewer;
};


struct GraphDrawableItem: public DrawableItem
{
    GraphDrawableItem(Graph* g, GraphViewer* v);

    virtual void draw();

    Graph* _graph;
    float _r, _g, _b;
};
#endif //DRAWABLE_H
