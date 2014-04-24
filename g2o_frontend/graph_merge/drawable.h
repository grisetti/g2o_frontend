#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <set>


class Edge;
class Graph;
class GraphViewer;
class RobotLaser;


struct DrawableItem
{
    DrawableItem(GraphViewer* viewer);
    virtual ~DrawableItem();

    virtual void draw() = 0;

    GraphViewer* _viewer;
};


struct GraphDrawableItem : public DrawableItem
{
    GraphDrawableItem(Graph* g, GraphViewer* v);
    ~GraphDrawableItem();

    virtual void draw();

    Graph* _graph;
    float _r, _g, _b;
};


struct LaserDrawableItem : public DrawableItem
{
    LaserDrawableItem(RobotLaser* l, GraphViewer* v);
    ~LaserDrawableItem();

    virtual void draw();

    RobotLaser* _laser;
    float _r, _g, _b;
};



struct AssociationDrawableItem : public DrawableItem
{
    AssociationDrawableItem(std::set<Edge*>* m, GraphViewer* v);
    ~AssociationDrawableItem();

    virtual void draw();

    std::set<Edge*>* _matches;
    float _r, _g, _b;
};

#endif //DRAWABLE_H
