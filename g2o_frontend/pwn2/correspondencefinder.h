#ifndef _CORRESPONDENCEFINDER_H_
#define _CORRESPONDENCEFINDER_H_

#include "depthimage.h"
#include "frame.h"

namespace pwn {

struct Correspondence {
  Correspondence(int referenceIndex_ = -1, int currentIndex_ = -1) {
    referenceIndex = referenceIndex_;
    currentIndex = currentIndex_;
  }
  int referenceIndex, currentIndex;
};

typedef std::vector<Correspondence> CorrespondenceVector;

class CorrespondenceFinder {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  CorrespondenceFinder();

  inline const CorrespondenceVector& correspondences() const { return _correspondences; }
  inline CorrespondenceVector& correspondences() { return _correspondences; }
  inline int numCorrespondences() const { return _numCorrespondences; }
  inline const Eigen::MatrixXi& currentIndexImage() const {return _currentIndexImage;}
  inline Eigen::MatrixXi& currentIndexImage() {return _currentIndexImage;}
  inline const Eigen::MatrixXi& referenceIndexImage() const {return _referenceIndexImage;}
  inline Eigen::MatrixXi& referenceIndexImage() {return _referenceIndexImage;}
  inline const DepthImage& currentDepthImage() const {return _currentDepthImage;}
  inline DepthImage& currentDepthImage() {return _currentDepthImage;}
  inline const DepthImage& referenceDepthImage() const {return _referenceDepthImage;}
  inline DepthImage& referenceDepthImage() {return _referenceDepthImage;}

  inline float squaredThreshold() const { return _squaredThreshold; }
  inline float inlierDistanceThreshold() const { return _inlierDistanceThreshold; }
  inline float flatCurvatureThreshold() const { return _flatCurvatureThreshold; }
  inline float inlierCurvatureRatioThreshold() const { return _inlierCurvatureRatioThreshold; }
  inline float inlierNormalAngularThreshold() const { return _inlierNormalAngularThreshold; }
  inline int rows() const { return _rows; }
  inline int cols() const { return _cols; }

  inline void setInlierDistanceThreshold(const float inlierDistanceThreshold_) {
    _inlierDistanceThreshold = inlierDistanceThreshold_;
    _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
  } 
  inline void setFlatCurvatureThreshold(const float flatCurvatureThreshold_) { _flatCurvatureThreshold = flatCurvatureThreshold_; }  
  inline void setInlierCurvatureRatioThreshold(const float inlierCurvatureRatioThreshold_) { _inlierCurvatureRatioThreshold = inlierCurvatureRatioThreshold_; }
  inline void setInlierNormalAngularThreshold(const float inlierNormalAngularThreshold_) { _inlierNormalAngularThreshold = inlierNormalAngularThreshold_; }

  inline void setSize(const int rows_, const int cols_) {
    if(_rows != rows_ || _cols != cols_) {
      _rows = rows_;
      _cols = cols_;
      _referenceIndexImage.resize(_rows, _cols);
      _currentIndexImage.resize(_rows, _cols);
    }
  }

  void compute(const Frame &referenceScene, const Frame &currentScene, Eigen::Isometry3f T);

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

}

#endif
