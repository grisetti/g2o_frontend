#include <QGLViewer/qglviewer.h>


class Viewer : public QGLViewer
{
public:
    Viewer();

protected :
  virtual void init();
  virtual void draw();
};
