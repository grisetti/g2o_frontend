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


GraphDrawableItem::GraphDrawableItem(Graph* g, GraphViewer* v) : DrawableItem(v)
{
    this->_graph = g;
    this->_r = 0.8;
    this->_g = 0;
    this->_b = 0;
}


GraphDrawableItem::~GraphDrawableItem()
{
    delete _graph;
}


void GraphDrawableItem::draw()
{
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


LaserDrawableItem::LaserDrawableItem(RobotLaser* l, GraphViewer* v) : DrawableItem(v)
{
    this->_laser = l;
    this->_r = 1;
    this->_g = 0;
    this->_b = 0;
}


LaserDrawableItem::~LaserDrawableItem()
{
    delete _laser;
}


void LaserDrawableItem::draw()
{
    RobotLaser::Vector2fVector points = _laser->floatCartesian();

    int k = 0;
    if(_laser->_maxRange && _laser->maxRange() >= 0 )
    {
        float r2 = _laser->maxRange();
        r2 *= r2;
        for(size_t i = 0; i < points.size(); i++)
        {
            if(points[i].squaredNorm() < r2)
            {
                points[k++]=points[i];
            }
        }
        points.resize(k);
    }


//    const OptimizableGraph::Vertex* v = dynamic_cast<const OptimizableGraph::Vertex*> (container);
//    if (! v) return false;

//    const Parameter* p = that->parameter();
//    const ParameterSE3Offset* oparam = dynamic_cast<const ParameterSE3Offset*> (p);

    // 	TODO
//    Eigen::Isometry3d offset = oparam->offset();
    //  Eigen::Isometry2d offset;
    // 	offset.setIdentity();

    glPushMatrix();
//    glMultMatrixd(offset.data());
    glColor4f(_r, _g, _b, 0.5f);
    glBegin(GL_POINTS);
    for(size_t i = 0; i < points.size(); i++)
    {
        glVertex3f(points[i].x(), points[i].y(), 0.f);
    }
    glEnd();
    glPopMatrix();
}


AssociationDrawableItem::AssociationDrawableItem(set<Edge*>* m, GraphViewer* v) : DrawableItem(v)
{
    this->_matches = m;
    this->_r = 0;
    this->_g = 0;
    this->_b = 1;
}


AssociationDrawableItem::~AssociationDrawableItem()
{
    delete _matches;
}


void AssociationDrawableItem::draw()
{
    glLineWidth(1.5);
    glColor3f(_r, _g, _b);
    glBegin(GL_LINES);
    for(set<Edge*>::const_iterator it = _matches->begin(); it != _matches->end(); it++)
    {
        Edge* e = *it;
        Node* n1 = e->_from;
        Node* n2 = e->_to;
        glVertex3f(n1->_pose.translation().x(), n1->_pose.translation().y(), 0);
        glVertex3f(n2->_pose.translation().x(), n2->_pose.translation().y(), 0);
    }
    glEnd();
}
