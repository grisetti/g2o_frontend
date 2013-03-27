#ifndef MAPQGLVIEWER_H
#define MAPQGLVIEWER_H

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


class MapQGLViewer: public QGLViewer
{

public:
    MapQGLViewer(QWidget *parent);
    virtual void init();
    virtual void draw();

    void drawVertex();
    void setDataPointer(const Vertices drawableVertices_, const Edges drawableEdges_);

    Vertices _drawableVertices;
    Edges _drawableEdges;

};

#endif // MAPQGLVIEWER_H
