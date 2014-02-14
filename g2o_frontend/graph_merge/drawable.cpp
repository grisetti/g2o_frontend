#include "drawable.h"
#include "graph.h"
#include "viewer.h"


using namespace std;



DrawableItem::DrawableItem(GraphViewer* viewer)
{
    this->_viewer = viewer;
    _viewer->addItem(this);
}


DrawableItem::~DrawableItem()
{
    if(_viewer)
    {
        _viewer->removeItem(this);
    }
}


GraphDrawableItem::GraphDrawableItem( Graph* g, GraphViewer* v): DrawableItem(v)
{
    _graph = g;
    this->_r = 0.8;
    this->_g = 0;
    this->_b = 0;
}


void GraphDrawableItem::draw()
{
//    cerr << "g: " << _graph << endl;
    glColor3f(_r, _g, _b);
    glPointSize(3);
    glBegin(GL_LINES);
    int numEdges = 0;
    for(EdgeMap::iterator it = _graph->_edges.begin(); it != _graph->_edges.end(); it++)
    {
        Edge* e = it->second;
        Node* n1 = e->_from;
        Node* n2 = e->_to;
        glVertex3f(n1->_pose.translation().x(), n1->_pose.translation().y(), 0);
        glVertex3f(n2->_pose.translation().x(), n2->_pose.translation().y(), 0);
        numEdges ++;
    }
    glEnd();
}
