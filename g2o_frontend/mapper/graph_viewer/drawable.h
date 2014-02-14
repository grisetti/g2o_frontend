#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <Eigen/Geometry>
#include <GL/gl.h>


class GraphQGLViewer;



class Drawable
{
public:
    Drawable();
    Drawable(Eigen::Isometry3f transformation_);

    inline virtual void setTransformation(Eigen::Isometry3f transformation_) { _transformation = transformation_; }
    inline virtual void setViewer(GraphQGLViewer *viewer_) { _viewer = viewer_; }
    inline virtual GraphQGLViewer* viewer() { return _viewer; }
    inline virtual Eigen::Isometry3f transformation() { return _transformation; }

    virtual void draw() {}


protected:
    Eigen::Isometry3f _transformation;
    GraphQGLViewer *_viewer;
};
#endif // DRAWABLE_H
