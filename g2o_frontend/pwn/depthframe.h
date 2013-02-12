#ifndef _SCENE_H_
#include "depthimage.h"
#include "pointwithnormal.h"
#include "gaussian3.h"
#include "pointwithnormalstatsgenerator";


struct Scene{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  PointWithNormalVector points;
  Gaussian3fVector gaussians;
  PointWithNormalSVDVector svds;
  
  void toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose,
		    float dmax = std::numeric_limits<float>::max());

  bool loadFromImage(std::string filename);
  void fromDepthImage(const DepthImage& image);
  void transform(const Eigen::Isometry3f& T);
protected: 
  void _updateStats(PointWithNormalStatistcsGenerator & generator, 
		    const Matrix3f& cameraMatrix, 
		    const Eigen::Isometry3f& cameraPose);
};

#endif
