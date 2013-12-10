#pragma once 

#include <Eigen/Geometry>
#include <GL/gl.h>
#include "gl_parameter.h"

namespace pwn {

  class GLParameterNormals: public GLParameter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GLParameterNormals();
    GLParameterNormals(float pointSize_, Eigen::Vector4f color_, float normalLength_);
    virtual ~GLParameterNormals() {}
 
    virtual void applyGLParameter();

    float pointSize() { return _pointSize; }
    void setPointSize(float pointSize_) { _pointSize = pointSize_; }
  
    Eigen::Vector4f color() { return _color; }
    void setColor(Eigen::Vector4f color_) { _color = color_; }
    
    float normalLength() { return _normalLength; }
    void setNormalLength(float normalLength_) { _normalLength = normalLength_; }

  protected:
    float _pointSize;
    Eigen::Vector4f _color;
    float _normalLength;
  };

}
