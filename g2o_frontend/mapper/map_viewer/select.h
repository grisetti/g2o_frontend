#include <QGLViewer/qglviewer.h>

class Viewer : public QGLViewer
{
public:
    Viewer();

protected :
  virtual void draw();
  virtual void drawWithNames();
//  virtual void postSelection(const QPoint& point);
  virtual void init();

private :
  qglviewer::Vec orig, dir, selectedPoint;
};
