#include "gl_parameter_cloud.h"

namespace pwn {

  GLParameterCloud::GLParameterCloud(int step) : GLParameter() {
    _parameterPoints = new GLParameterPoints(1.0f, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
    _parameterNormals = new GLParameterNormals(1.0f, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    _parameterCovariances = new GLParameterCovariances(1.0f, 
						       Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f),
						       0.02f, 0.0f);
    _parameterCorrespondences = new GLParameterCorrespondences(1.0f, Eigen::Vector4f(1.0f, 0.0f, 1.0f, 1.0f), 0.0f);
  
    _parameterPoints->setStep(step);
    _parameterNormals->setStep(step);
    _parameterCovariances->setStep(step);
    _parameterCorrespondences->setStep(step);  
  };

  GLParameterCloud::~GLParameterCloud() {
    delete _parameterPoints;
    delete _parameterNormals;
    delete _parameterCovariances;    
    delete _parameterCorrespondences;
  }

}
