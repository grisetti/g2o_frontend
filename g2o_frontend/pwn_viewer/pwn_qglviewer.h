#ifndef PWN_QGLVIEWER
#define PWN_QGLVIEWER
#include <QGLViewer/qglviewer.h>
#include <vector>
#include "drawable.h"

class PWNQGLViewer : public QGLViewer {
 public:
  PWNQGLViewer(QWidget* parent=0, const QGLWidget* shareWidget=0, Qt::WFlags flags=0);
  virtual void init();
  virtual void draw();
  virtual void addDrawable(Drawable *d);
  virtual void clearDrawableList() { _drawableList.clear(); }
  virtual std::vector<Drawable*> drawableList() { return _drawableList; }   
  GLuint ellipsoidDrawList() { return _ellipsoidDrawList; }
  GLuint pyramidDrawList() { return _ellipsoidDrawList; }
 
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
