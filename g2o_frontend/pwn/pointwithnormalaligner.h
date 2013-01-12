#ifndef _POINTWITHNORMAL_ALIGNER_
#define _POINTWITHNORMAL_ALIGNER_

#include "pointwithnormal.h"
#include "depthimage.h"
#include "pointwithnormalstatsgenerator.h"
#include "g2o_frontend/dm_optimization/dm_math.h"

class PointWithNormalAligner{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Correspondence{
    Correspondence(int i1_=-1, int i2_=-1) {
      i1=i1_;
      i2=i2_;
    }
    int i1, i2;
  };
  typedef std::vector<Correspondence> CorrespondenceVector;
    

  PointWithNormalAligner();

  int align(float& error, Eigen::Isometry3f& X);

  inline void setImageSize(int r, int c) {_rows = r; _cols = c; _cameraSet=false;}
  inline int imageCols() const { return _cols;}
  inline int imageRows() const { return _rows;}
  inline int scaledImageCols() const { return _refZbuffer.cols();}
  inline int scaledImageRows() const { return _refZbuffer.rows();}

// sets the cloud and conditions tthe covariances accordingly
  void setReferenceCloud(const PointWithNormalVector*  refPoints, const PointWithNormalSVDVector* refSVDs); 
  // sets the cloud and conditions tthe covariances accordingly
  void setCurrentCloud(const PointWithNormalVector*  currentPoints, const PointWithNormalSVDVector* currentSVDs); 

  const PointWithNormalVector* referenceCloud() const {return _refPoints;}
  const PointWithNormalSVDVector* referenceSVDs() const {return _refSVDs;}
  const PointWithNormalVector* currentCloud() const {return _currPoints;}
  const PointWithNormalSVDVector* currentSVDs() const {return _currSVDs;}

  
  inline float scale() const {return _scale;}
  inline void setScale(float scale_) {_scale = scale_; _cameraSet = false;}

  inline float inlierNormalAngularThreshold() const {return _inlierNormalAngularThreshold;}
  inline void setInlierNormalAngularThreshold(float inlierNormalAngularThreshold_) {_inlierNormalAngularThreshold = inlierNormalAngularThreshold_;}

  inline float inlierDistanceThreshold() const {return _inlierDistanceThreshold;}
  inline void setInlierDistanceThreshold(float inlierDistanceThreshold_) {_inlierDistanceThreshold = inlierDistanceThreshold_;}

  inline float inlierCurvatureRatioThreshold() const {return _inlierCurvatureRatioThreshold;}
  inline void setInlierCurvatureRatioThreshold(float inlierCurvatureRatioThreshold_) {_inlierCurvatureRatioThreshold = inlierCurvatureRatioThreshold_;}

  inline float inlierMaxChi2() const {return _inlierMaxChi2;}
  inline void  setInlierMaxChi2(float inlierMaxChi2_) {_inlierMaxChi2 = inlierMaxChi2_;}

  inline float flatCurvatureThreshold() const {return _flatCurvatureThreshold;}
  inline void  setFlatCurvatureThreshold(float flatCurvatureThreshold_) {_flatCurvatureThreshold = flatCurvatureThreshold_; _omegasSet = false;}
  
  inline Eigen::Vector3f flatOmegaDiagonal() const {return _flatOmegaDiagonal;}
  inline void setFlatOmegaDiagonal (const Eigen::Vector3f& flatOmegaDiagonal_) {_flatOmegaDiagonal = flatOmegaDiagonal_; _omegasSet = false;}
  
  inline int outerIterations() const {return _outerIterations;}
  inline void setOuterIterations(int outerIterations_) {_outerIterations = outerIterations_;}

  inline int nonLinearIterations() const {return _nonLinearIterations;}
  inline void setNonLinearIterations(int nonLinearIterations_) {_nonLinearIterations = nonLinearIterations_;}

  inline int linearIterations() const {return _linearIterations;}
  inline void setLinearIterations(int linearIterations_) {_linearIterations = linearIterations_;}

  inline const Eigen::Matrix3f& cameraMatrix() const { return _cameraMatrix;}
  inline void setCameraMatrix(const Eigen::Matrix3f cameraMatrix_)  { _cameraMatrix = cameraMatrix_; _cameraSet = false;}

  inline const CorrespondenceVector& correspondences() const {return _correspondences;}
  inline int numCorrespondences() const {return _numCorrespondences;}
  
  inline float lambda()  const {return _lambda;}
  inline void setLambda(float lambda_)  {_lambda=lambda_;}

  inline void setDebug( bool debug_) {_debug = debug_;}
  inline bool debug() const {return _debug;}

  inline int minInliers() const {return _minInliers;}
  inline void setMinInliers(int minInliers_) {_minInliers = minInliers_;}
protected:
  // size of the original image
  int _rows,  _cols;

  void _updateCamera();
  void _updateOmegas();

  int _computeErrorAndInliers(float& error) const;
  int _nonLinearUpdate(float& error);
  int _linearUpdate(float& error);

  int _constructLinearSystemQT(Matrix6f& H, Vector6f&b, float& error);
  int _constructLinearSystemRT(Matrix12f& H, Vector12f&b, float& error);
  int _constructLinearSystemT(Eigen::Matrix3f& H, Eigen::Vector3f&b, float& error);
  // parameters for data association
  float _scale;
  float _inlierNormalAngularThreshold;
  float _inlierDistanceThreshold;
  float _inlierMaxChi2;
  float _inlierCurvatureRatioThreshold;
  float _flatCurvatureThreshold;
  
  // parameters for optimization
  Eigen::Vector3f _flatOmegaDiagonal;
  float _lambda;

  // parameters for optimization
  int _outerIterations;
  int _nonLinearIterations;
  int _linearIterations;
  int _minInliers;
  Eigen::Matrix3f _cameraMatrix;
  Eigen::Matrix3f _scaledCameraMatrix;
  
  bool _cameraSet;
  bool _omegasSet;

  const PointWithNormalVector *_refPoints, *_currPoints;
  const PointWithNormalSVDVector *_refSVDs, *_currSVDs;
  Matrix6fVector _currOmegas;

  DepthImage _refZbuffer, _currZbuffer;
  Eigen::MatrixXi _refIndexImage, _currIndexImage;
  CorrespondenceVector _correspondences;
  int _numCorrespondences;

  Eigen::Isometry3f _T;
  bool _debug;
};

#endif
