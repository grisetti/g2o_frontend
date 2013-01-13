#include "drawable_correspondences.h"
#include "gl_parameter_correspondences.h"

DrawableCorrespondences::DrawableCorrespondences() : Drawable() {
  GLParameterCorrespondences* correspondencesParameter = new GLParameterCorrespondences();
  _parameter = (GLParameter*)correspondencesParameter;
  _correspondences = 0;
  _numCorrespondences = 0;
  _points1 = 0;
  _points2 = 0;
}

DrawableCorrespondences::DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_, int step_, int numCorrespondences_, PointWithNormalAligner::CorrespondenceVector *correspondences_) : Drawable(transformation_, step_) {
  setParameter(parameter_);
  _numCorrespondences = numCorrespondences_;
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
    correspondencesParameter->applyGLParameter();
    glBegin(GL_LINES);
    for (int i = 0; i < _numCorrespondences; i += _step) {
      const PointWithNormalAligner::Correspondence& correspondence = _correspondences->at(i);
      Eigen::Vector3f p0 = _points1->at(correspondence.i1).point();
      Eigen::Vector3f p1 = _transformation * (_points2->at(correspondence.i2).point());
      glVertex3f(p1[0], p1[1], p1[2]);
      glVertex3f(p0[0], p0[1], p0[2]);
    }
    glEnd();
    glPopMatrix();
  }
}
