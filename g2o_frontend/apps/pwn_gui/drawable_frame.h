#ifndef _DRAWABLE_FRAME_OLD_H_
#define _DRAWABLE_FRAME_OLD_H_

#include "g2o/types/slam3d/types_slam3d.h"

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

namespace pwn{

/** this thing keeps the state of the parameters of the viewer. 
    We create the parameters only once and we bound these parameters 
    to each drawable scene we make*/

struct DrawableFrameParameters : public GLParameter{
  DrawableFrameParameters(int step = 1);
  ~DrawableFrameParameters();
  void applyGLParameter();

  GLParameterPoints *_pPoints;
  GLParameterNormals *_pNormals;
  GLParameterCovariances *_pCovariances;
  GLParameterCorrespondences *_pCorrespondences;
};


struct DrawableFrame : public Drawable {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Eigen::Isometry3f _localTransform;
  Frame* _frame;
  DrawableFrameParameters * _parameters;
  DrawablePoints *_dPoints;
  DrawableNormals *_dNormals;
  DrawableCovariances *_dCovariances;
  DrawableCorrespondences *_dCorrespondences;
  DrawableFrame* _previousDrawableFrame;


  DrawableFrame(const Eigen::Isometry3f& transformation_ = Eigen::Isometry3f::Identity(), 
		DrawableFrameParameters* parameters_=0, 
		Frame* frame_=0);

  virtual ~DrawableFrame();


  virtual void setViewer(PWNQGLViewer *viewer_);

  virtual bool setParameter(GLParameter *parameter_);

  virtual GLParameter* parameter();

  void clearDrawableObjects();

  void constructDrawableObjects();

  void setFrame(Frame* f);

  Frame* frame() const;

  const Eigen::Isometry3f& localTransformation() const;

  void setLocalTransformation(const Eigen::Isometry3f& localTransform_);

  void draw();

  //HEAVY TREMENDOUS HACK
  g2o::VertexSE3* _vertex; 
  bool _hasImu;
  Eigen::Isometry3f _imuMean;
  Matrix6f _imuInformation;
};

} // end namespace pwn
#endif
