#ifndef GL_PARAMETER
#define GL_PARAMETER

#include <GL/gl.h>
#include <QObject>

class GLParameter : public QObject{
  Q_OBJECT;
 public:
  GLParameter();
  virtual void applyGLParameter() = 0;
  inline int step() { return _step; }
  inline bool isShown() const {return _show;}
public slots:
  inline void setShow(bool show_) {_show = show_;}
  inline void setStep(int step_) { _step = step_; }
protected:
  int _step;
  bool _show;
};

#endif
