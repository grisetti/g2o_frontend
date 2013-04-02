#ifndef GRAPHQGLVIEWER_H
#define GRAPHQGLVIEWER_H

#include <QGLViewer/qglviewer.h>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"



/**
 * \brief helper for setting up a camera for qglviewer
 */
class StandardCamera : public qglviewer::Camera
{

public:
    StandardCamera() : _standard(true) {}

    float zNear() const
    {
        if(_standard)
            return 0.001f;
        else
            return Camera::zNear();
    }

    float zFar() const
    {
        if(_standard)
            return 10000.0f;
        else
            return Camera::zFar();
    }

    bool standard() const {return _standard;}
    void setStandard(bool s) { _standard = s;}

private:
    bool _standard;
};



typedef std::vector<g2o::HyperGraph::Vertex*> Vertices;
typedef std::vector<g2o::HyperGraph::Edge*> Edges;


class GraphQGLViewer: public QGLViewer
{

public:
    GraphQGLViewer(QWidget *parent);
    virtual void init();
    virtual void draw();

    void drawVertex();
    void drawGraph(const Vertices& v);
    void drawReferenceGraph();
    void drawCurrentGraph();


    inline void setDrawableVertex(g2o::HyperGraph::Vertex* v)
    {
        _referenceGraphVertices.push_back(v);
        drawReferenceGraph();
    }


    inline void setReferenceGraph(Vertices v)
    {
        _referenceGraphVertices = v;
        drawReferenceGraph();
    }


    inline void cancelReferenceGraph()
    {
        _referenceGraphVertices.clear();
        _referenceGraphEdges.clear();
        drawReferenceGraph();
    }


    inline void setAlpha(float a) { _alpha = a; }

    inline void setColor(float r, float g, float b)
    {
        _red = r;
        _green = g;
        _blue = b;
    }

    inline void setDepth(float d) { _depth = d; }


    Vertices _drawableVertices;
    Edges _drawableEdges;

    Vertices _referenceGraphVertices;
    Edges _referenceGraphEdges;

    float _alpha;
    float _color;
    float _depth;

    float _red;
    float _green;
    float _blue;
};

#endif // GRAPHQGLVIEWER_H
