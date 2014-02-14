#include "drawable_vertex.h"
#include "graph_qglviewer.h"


using namespace std;



DrawableVertex::DrawableVertex() : Drawable()
{
    _vertex = 0;
    _payload = 0;
}


DrawableVertex::DrawableVertex(g2o::VertexSE2* vertex_) : Drawable()
{
    _vertex = vertex_;
    _payload = 0;
}


void DrawableVertex::draw()
{
    glColor4f(1.0f, 1.0f, 1.0f, 1.f);
    glTranslatef((float) _vertex->estimate().translation().x(), (float) _vertex->estimate().translation().y(), 0.f);
    glRotatef((float) RAD2DEG(_vertex->estimate().rotation().angle()), 0.f, 0.f, 1.f);
    glCallList(_viewer->arrowDrawList());
    if(_payload)
    {
        _payload->draw();
    }
    else
    {
        cout << "Vertex payload not set" << endl;
    }
}
