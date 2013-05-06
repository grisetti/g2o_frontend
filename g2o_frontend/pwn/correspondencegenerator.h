#ifndef _CORRESPONDENCEGENERATOR_H_
#define _CORRESPONDENCEGENERATOR_H_

#include "depthimage.h"
#include "homogeneouspoint3fscene.h"

struct Correspondence{
  Correspondence(int referenceIndex_ = -1, int currentIndex_ = -1) {
    referenceIndex = referenceIndex_;
    currentIndex = currentIndex_;
  }
  int referenceIndex, currentIndex;
};

typedef std::vector<Correspondence> CorrespondenceVector;

class CorrespondenceGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  CorrespondenceGenerator() {
  _inlierDistanceThreshold = 3.0f;  
  _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
  _inlierNormalAngularThreshold = cos(M_PI/6);
  _flatCurvatureThreshold = 0.02f;
  _inlierCurvatureRatioThreshold = 1.3f;
  _numCorrespondences = 0;
  _rows = 0;
  _cols = 0;
}

  inline int numCorrespondences() { return _numCorrespondences; }
  inline CorrespondenceVector& correspondences() { return _correspondences; }
  inline float squaredThreshold() { return _squaredThreshold; }
  inline float inlierDistanceThreshold() { return _inlierDistanceThreshold; }
  inline float flatCurvatureThreshold() { return _flatCurvatureThreshold; }
  inline float inlierCurvatureRatioThreshold() { return _inlierCurvatureRatioThreshold; }
  inline float inlierNormalAngularThreshold() { return _inlierNormalAngularThreshold; }
  inline Eigen::MatrixXi& referenceIndexImage() { return _referenceIndexImage; }
  inline Eigen::MatrixXi& currentIndexImage() { return _currentIndexImage; }
  inline DepthImage& referenceDepthImage() { return _referenceDepthImage; }
  inline DepthImage& currentDepthImage() { return _currentDepthImage; }
  inline int rows() { return _rows; }
  inline int cols() { return _cols; }

  inline void setInlierDistanceThreshold(float inlierDistanceThreshold_) {
    _inlierDistanceThreshold = inlierDistanceThreshold_;
    _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
  } 
  inline void setCorrespondences(CorrespondenceVector &correspondences_) { _correspondences = correspondences_; }
  inline void setFlatCurvatureThreshold(float flatCurvatureThreshold_) { _flatCurvatureThreshold = flatCurvatureThreshold_; }  
  inline void setInlierCurvatureRatioThreshold(float inlierCurvatureRatioThreshold_) { _inlierCurvatureRatioThreshold = inlierCurvatureRatioThreshold_; }
  inline void setInlierNormalAngularThreshold(float inlierNormalAngularThreshold_) { _inlierNormalAngularThreshold = inlierNormalAngularThreshold_; }
  inline void setReferenceIndexImage(Eigen::MatrixXi &referenceIndexImage_) { _referenceIndexImage = referenceIndexImage_; }
  inline void setCurrentIndexImage(Eigen::MatrixXi &currentIndexImage_) { _currentIndexImage = currentIndexImage_; }
  inline void setReferenceDepthImage(DepthImage &referenceDepthImage_) { _referenceDepthImage = referenceDepthImage_; }
  inline void setCurrentDepthImage(DepthImage &currentDepthImage_) { _currentDepthImage = currentDepthImage_; }
  inline void setSize(int rows_, int cols_) {
    if(_rows != rows_ || _cols != cols_) {
      _rows = rows_;
      _cols = cols_;
      _referenceIndexImage.resize(_rows, _cols);
      _currentIndexImage.resize(_rows, _cols);
    }
  }

  void compute(HomogeneousPoint3fScene &referenceScene, HomogeneousPoint3fScene &currentScene, Eigen::Isometry3f T);

 protected:
  float _squaredThreshold;
  float _inlierNormalAngularThreshold;
  float _flatCurvatureThreshold;
  float _inlierCurvatureRatioThreshold;
  float _inlierDistanceThreshold;
  
  int _numCorrespondences;
  CorrespondenceVector _correspondences;

  int _rows, _cols;
  Eigen::MatrixXi _referenceIndexImage, _currentIndexImage;
  DepthImage _referenceDepthImage, _currentDepthImage;
};

#endif
