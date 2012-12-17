#ifndef DM_QGLVIEWER
#define DM_QGLVIEWER
#include <QGLViewer/qglviewer.h>
#include <vector>
#include "drawable.h"

class DMQGLViewer : public QGLViewer {
 public:
  DMQGLViewer(QWidget* parent=0, const QGLWidget* shareWidget=0, Qt::WFlags flags=0);
  virtual void init();
  virtual void draw();
  virtual void addDrawable(Drawable *d);
 
 protected:
  // Number of draw lists.
  int _numDrawLists;
  // Draw list to generate an ellipsoid.
  GLuint _ellipsoidDrawList;
  // Draw list to generate a pyramid.
  GLuint _pyramidDrawList;
  // Vector of Drawable objects to draw.
  std::vector<Drawable*> _drawableList;  
};

#endif
