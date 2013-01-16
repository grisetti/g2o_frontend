#include "drawable_normals.h"
#include "gl_parameter_normals.h"

DrawableNormals::DrawableNormals() : Drawable(){
  GLParameterNormals* normalsParameter = new GLParameterNormals();
  _parameter = (GLParameter*)normalsParameter;
  _normals = 0;
}

DrawableNormals::DrawableNormals(Eigen::Isometry3f transformation_, GLParameter *parameter_, int step_, PointWithNormalVector *normals_) : Drawable(transformation_, step_) {
  setParameter(parameter_);
  _normals = normals_;
}

bool DrawableNormals::setParameter(GLParameter *parameter_) {
  GLParameterNormals* normalsParameter = (GLParameterNormals*)parameter_;
  if (normalsParameter == 0) {
    _parameter = 0;
    return false;
  }
  _parameter = parameter_;
  return true;
}

// Drawing function of the class object.
void DrawableNormals::draw() {
  GLParameterNormals* normalsParameter = (GLParameterNormals*)_parameter;
  if (_normals && normalsParameter && normalsParameter->normalLength() > 0.0f) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    normalsParameter->applyGLParameter();
    glLineWidth(1.0);
    float normalLength = normalsParameter->normalLength();
    glBegin(GL_LINES);
    for (size_t i = 0; i < _normals->size(); i += _step) {
      const Vector6f &p = (*_normals)[i];
      glVertex3f(p[0], p[1], p[2]);
      glVertex3f(p[0] + p[3]*normalLength,
		 p[1] + p[4]*normalLength, 
		 p[2] + p[5]*normalLength);
    }
    glEnd();
    glPopMatrix();
  }
}
