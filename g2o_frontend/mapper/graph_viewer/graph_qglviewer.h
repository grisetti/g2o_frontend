#ifndef GRAPHQGLVIEWER_H
#define GRAPHQGLVIEWER_H

#include <QGLViewer/qglviewer.h>
#include <vector>
#include "drawable.h"



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


class GraphQGLViewer : public QGLViewer
{
public:
    GraphQGLViewer(QWidget* parent);
    virtual void init();
    virtual void draw();
    virtual void addDrawable(Drawable* d);

    inline void popBack() { _drawableList.pop_back(); }
    inline void clearDrawableList() { _drawableList.clear(); }
    inline size_t drawableListSize() {return _drawableList.size(); }

    inline std::vector<Drawable*>& drawableList() { return _drawableList; }
    inline const std::vector<Drawable*>& drawableList() const { return _drawableList; }

    inline GLuint arrowDrawList() { return _arrowDrawList; }

protected:
    GLuint _numDrawLists;
    GLuint _arrowDrawList;
    std::vector<Drawable*> _drawableList;
};
#endif // GRAPHQGLVIEWER_H
