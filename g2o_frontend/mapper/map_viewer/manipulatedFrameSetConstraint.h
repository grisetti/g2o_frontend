#ifndef MANIPULATED_FRAME_SET_CONSTRAINTS_H
#define MANIPULATED_FRAME_SET_CONSTRAINTS_H

#include "QGLViewer/constraint.h"
#include "drawable_object.h"

class ManipulatedFrameSetConstraint : public qglviewer::Constraint
{
public:
  void clearSet();
  void addObjectToSet(DrawableObject* o);

  virtual void constrainTranslation(qglviewer::Vec &translation, qglviewer::Frame *const frame);
  virtual void constrainRotation(qglviewer::Quaternion &rotation, qglviewer::Frame *const frame);

private:
  QList<DrawableObject*> objects_;
};
#endif //MANIPULATED_FRAME_SET_CONSTRAINTS_H
