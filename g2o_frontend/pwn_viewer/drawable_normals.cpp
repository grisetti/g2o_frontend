#include "drawable_normals.h"
#include "gl_parameter_normals.h"

DrawableNormals::DrawableNormals() : Drawable(){
  GLParameterNormals* normalsParameter = new GLParameterNormals();
  _parameter = (GLParameter*)normalsParameter;
}

DrawableNormals::DrawableNormals(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, const HomogeneousPoint3fVector &points_, const HomogeneousNormal3fVector &normals_) : Drawable(transformation_) {
  setParameter(parameter_);
  _points = points_;
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
  GLParameterNormals* normalsParameter = dynamic_cast<GLParameterNormals*>(_parameter);
  if (_points.size() > 0 &&
      _normals.size() > 0 && 
      normalsParameter &&
      normalsParameter->isShown() && 
      normalsParameter->normalLength() > 0.0f) {
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    normalsParameter->applyGLParameter();
    glLineWidth(1.0);
    float normalLength = normalsParameter->normalLength();
    glBegin(GL_LINES);
    for (size_t i = 0; i < _normals.size(); i += normalsParameter->step()) {
      const HomogeneousPoint3f &p = _points[i];
      const HomogeneousNormal3f &n = _normals[i];
      glVertex3f(p[0], p[1], p[2]);
      glVertex3f(p[0] + n[0]*normalLength,
		 p[1] + n[1]*normalLength, 
		 p[2] + n[2]*normalLength);
    }
    glEnd();
    glPopMatrix();
  }
}
