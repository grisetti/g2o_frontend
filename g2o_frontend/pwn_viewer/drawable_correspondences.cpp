#include "drawable_correspondences.h"
#include "gl_parameter_correspondences.h"
#include <iostream>

using namespace std;

DrawableCorrespondences::DrawableCorrespondences() : Drawable() {
  GLParameterCorrespondences* correspondencesParameter = new GLParameterCorrespondences();
  _transformation = Eigen::Isometry3f::Identity();
  _parameter = (GLParameter*)correspondencesParameter;
  _numCorrespondences = 0;
}

DrawableCorrespondences::DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, 
						 HomogeneousPoint3fVector &referencePoints_, HomogeneousPoint3fVector &currentPoints_, 
						 CorrespondenceVector &correspondences_) {
  _transformation = transformation_;
  setParameter(parameter_);
  _numCorrespondences = numCorrespondences_;
  _referencePoints = referencePoints_;
  _currentPoints = currentPoints_;
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
  GLParameterCorrespondences* correspondencesParameter = dynamic_cast<GLParameterCorrespondences*>(_parameter);
  if (_referencePoints.size() > 0 &&
      _currentPoints.size() > 0 &&
      _correspondences.size() > 0 && 
      correspondencesParameter && 
      correspondencesParameter->isShown() && 
      correspondencesParameter->lineWidth() > 0.0f) {
    
    glPushMatrix();
    correspondencesParameter->applyGLParameter();
    glBegin(GL_LINES);
    for (int i = 0; i < _numCorrespondences; i += correspondencesParameter->step()) {
      const Correspondence& correspondence = _correspondences[i];
      HomogeneousPoint3f referencePoint = _referencePoints[correspondence.referenceIndex];
      HomogeneousPoint3f currentPoint = _transformation * (_currentPoints[correspondence.currentIndex]);
      glVertex3f(referencePoint.x(), referencePoint.y(), referencePoint.z());
      glVertex3f(currentPoint.x(), currentPoint.y(), currentPoint.z());
    }
    glEnd();
    glPopMatrix();
  }
}
