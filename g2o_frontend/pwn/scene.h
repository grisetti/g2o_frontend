#ifndef _SCENE_H_
#define _SCENE_H_
#include "depthimage.h"
#include "pointwithnormal.h"
#include "gaussian3.h"
#include "pointwithnormalstatsgenerator.h"


struct Scene{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class SceneManipulator;
  inline const PointWithNormalVector& points() const { return _points; }
  inline const Gaussian3fVector& gaussians() const {return _gaussians;}
  inline const PointWithNormalSVDVector& svds() const {return _svds;}
  void toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose,
		    float dmax = std::numeric_limits<float>::max());

  void subScene(Scene& partial, const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose,
		int rows=480, int columns=640, float scale=1, float dmax = std::numeric_limits<float>::max());

  void transform(const Eigen::Isometry3f& T);
  void add(const Scene& scene, const Eigen::Isometry3f& T=Eigen::Isometry3f::Identity());
  void clear();
  size_t size() const {return _points.size();}
  //protected: 
  void _updatePointsFromGaussians(bool eraseNormals=true);
  void _updateSVDsFromPoints(PointWithNormalStatistcsGenerator & generator, 
			    const Eigen::Matrix3f& cameraMatrix, 
			    const Eigen::Isometry3f& cameraPose=Eigen::Isometry3f::Identity(),
			    int r=480, int c=640,
			    float dmax = std::numeric_limits<float>::max());
  void _suppressNoNormals();
  PointWithNormalVector _points;
  Gaussian3fVector _gaussians;
  PointWithNormalSVDVector _svds;

};

struct DepthFrame : public Scene{
  friend class SceneManipulator;
  DepthFrame();
  inline const DepthImage& image() const {return _image;}
  inline const Eigen::Matrix3f& cameraMatrix() const {return _cameraMatrix;}
  float maxDistance() const { return _maxDistance;}
  float baseline() const { return _baseline;}
  void setImage(const DepthImage& image_);
  //protected:
  void _updateGaussiansFromImage();
  DepthImage _image;
  Eigen::Matrix3f _cameraMatrix;
  float _maxDistance;
  float _baseline;
};

// class SceneManipulator{
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//   void SceneManipulator(PointWithNormalStatistcsGenerator* normalGenerator_,
// 			PointWithNormalAligner* _aligner);
 
//   inline PointWithNormalAligner* aligner() const {return _aligner;}
//   inline PointWithNormalStatistcsGenerator* normalGenerator() const {return _normalGenerator;}

//   DepthFrame* loadDepthFrame(const std::string& filename);
//   protected:
//   Eigen::MatrixXf _zBuffer;
//   PointWithNormalStatistcsGenerator* _normalGenerator;
//   PointWithNormalAligner* _aligner;
// }
#endif
