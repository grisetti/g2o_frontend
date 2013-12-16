#pragma once

#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/pwn_imageview.h"

#include "g2o_frontend/pwn_viewer/drawable_points.h"
#include "g2o_frontend/pwn_viewer/drawable_normals.h"
#include "g2o_frontend/pwn_viewer/drawable_covariances.h"
#include "g2o_frontend/pwn_viewer/drawable_correspondences.h"

#include "g2o_frontend/pwn_viewer/gl_parameter.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_points.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_normals.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_covariances.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_correspondences.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_cloud.h"

#include "g2o/types/slam3d/types_slam3d.h"

namespace pwn {

  class DrawableCloud : public Drawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DrawableCloud(const Eigen::Isometry3f &transformation_ = Eigen::Isometry3f::Identity(), 
		  GLParameter *parameter_ = 0, Cloud *cloud_ = 0);
    virtual ~DrawableCloud() { clearDrawableObjects(); }

    const Eigen::Isometry3f& localTransformation() const { return _localTransform; }
    void setLocalTransformation(const Eigen::Isometry3f &localTransform_) { _localTransform = localTransform_; }

    virtual GLParameter* parameter() { return _parameter; } 
    virtual bool setParameter(GLParameter *parameter_);

    Cloud* cloud() const { return _cloud; }
    void setCloud(Cloud *cloud_);

    DrawableCorrespondences* drawableCorrespondences() { return _drawableCorrespondences; }
    void setDrawableCorrespondences(DrawableCorrespondences* drawableCorrespondences_) {
      if(_drawableCorrespondences)
	delete _drawableCorrespondences;
      _drawableCorrespondences = drawableCorrespondences_;
    }

    DrawablePoints* drawablePoints() { return _drawablePoints; }
    DrawableNormals* drawableNormals() { return _drawableNormals; }
    DrawableCovariances* drawableCovariances() { return _drawableCovariances; }
  
    void clearDrawableObjects();
    void constructDrawableObjects();

    void draw();

  protected:
    Eigen::Isometry3f _localTransform;
    Cloud *_cloud;
    DrawableCloud* _previousDrawableCloud;
    GLParameterCloud * _parameter;
    DrawablePoints *_drawablePoints;
    DrawableNormals *_drawableNormals;
    DrawableCovariances *_drawableCovariances;
    DrawableCorrespondences *_drawableCorrespondences;
  };

}
