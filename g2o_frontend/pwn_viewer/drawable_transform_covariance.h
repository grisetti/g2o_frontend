#pragma once

#include "gl_parameter_transform_covariance.h"
#include "drawable.h"

namespace pwn {

  class DrawableTransformCovariance : public Drawable {
  public:
    DrawableTransformCovariance();
    DrawableTransformCovariance(Eigen::Isometry3f transformation_, GLParameter *parameter_, Eigen::Matrix3f covariance_, Eigen::Vector3f mean_);
    virtual ~DrawableTransformCovariance() { 
      glDeleteLists(_covarianceDrawList, 1); 
      glDeleteLists(_sphereDrawList, 1); 
    }

    virtual GLParameter* parameter() { return _parameter; }
    virtual Eigen::Matrix3f covariance() { return _covariance; }
    virtual Eigen::Vector3f mean() { return _mean; }
    inline GLuint covarianceDrawList() { return _covarianceDrawList; }
    inline GLuint sphereDrawList() { return _sphereDrawList; }

    virtual bool setParameter(GLParameter *parameter_);
    virtual void setCovariances(Eigen::Matrix3f covariance_) { 
      _covariance = covariance_; 
      updateCovarianceDrawList();
    }
    virtual void setMean(Eigen::Vector3f mean_) { 
      _mean = mean_; 
      updateCovarianceDrawList();
    }

    virtual void draw();
    void updateCovarianceDrawList();

  protected:
    GLParameterTransformCovariance *_parameter;
    Eigen::Matrix3f _covariance;
    Eigen::Vector3f _mean;
    GLuint _covarianceDrawList; 
    GLuint _sphereDrawList; 
  };  

}
