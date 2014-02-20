#ifndef GRAPH_VIEWER_H
#define GRAPH_VIEWER_H

#include <set>
#include <QGLViewer/qglviewer.h>


class StandardCamera : public qglviewer::Camera
{
public:
    StandardCamera();

    float zNear() const;
    float zFar() const;

    inline void setStandard(bool s) { _standard = s; }
    inline bool standard() const { return _standard; }

protected:
    bool _standard;
};


class DrawableItem;
class Graph;

class GraphViewer: public QGLViewer
{
public:
    GraphViewer(QWidget *parent = 0,  const QGLWidget *shareWidget = 0, Qt::WFlags flags = 0);
    virtual ~GraphViewer();
    virtual void init();
    void draw();

    inline void addItem(DrawableItem* d) { _items.insert(d); }
    inline void removeItem(DrawableItem* d) { _items.erase(d); }

    Graph* _graph;
    std::set<DrawableItem*> _items;
};
#endif //GRAPH_VIEWER_H
