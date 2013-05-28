#ifndef GL_PARAMETER
#define GL_PARAMETER

#include <GL/gl.h>
#include <QObject>

namespace pwn {

class GLParameter : public QObject {
  Q_OBJECT;
 public:
  GLParameter();
  virtual ~GLParameter() {}

  virtual void applyGLParameter() = 0;

  inline int step() { return _step; }
  inline bool show() const { return _show; }

  inline void setStep(int step_) { _step = step_; }  
  inline void setShow(bool show_) { _show = show_; }
  
 protected:
  int _step;
  bool _show;
};

}

#endif
