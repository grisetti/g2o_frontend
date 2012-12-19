#include "drawable_correspondences.h"
#include "gl_parameter_correspondences.h"

DrawableCorrespondences::DrawableCorrespondences() : Drawable() {
  GLParameterCorrespondences* correspondencesParameter = new GLParameterCorrespondences();
  _parameter = (GLParameter*)correspondencesParameter;
  _correspondences = 0;
}

DrawableCorrespondences::DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_, int step_, CorrespondenceVector *correspondences_) : Drawable(transformation_, step_) {
  setParameter(parameter_);
  _correspondences = correspondences_;
}

bool DrawableCorrespondences::setParameter(GLParameter *parameter_) {
  GLParameterCorrespondences* correspondencesParameter = (GLParameterCorrespondences*)parameter_;
  if (correspondencesParameter == 0) {
    _parameter = 0;
    return false;
  }
  _parameter = parameter_;
  return true;
}

// Drawing function of the class object.
void DrawableCorrespondences::draw() {
  GLParameterCorrespondences* correspondencesParameter = (GLParameterCorrespondences*)_parameter;
   if (_correspondences && correspondencesParameter && correspondencesParameter->lineWidth() > 0.0f) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    correspondencesParameter->applyGLParameter();
    glBegin(GL_LINES);
    for (size_t i = 0; i < _correspondences->size(); i += _step) {
      const Vector6f& p1 = *((*_correspondences)[i].p1);
      const Vector6f& p2 = *((*_correspondences)[i].p2);
      glVertex3f(p1[0], p1[1], p1[2]);
      glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();
    glPopMatrix();
  }
}
