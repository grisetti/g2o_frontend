#ifndef _POINTWITHNORMAL_ALIGNER_
#define _POINTWITHNORMAL_ALIGNER_

#include "pointwithnormal.h"
#include "depthimage.h"
#include "pointwithnormalstatsgenerator.h"
#include "scene.h"
//#include "g2o_frontend/basemath/bm_se3.h"
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
  int align(float& error, Eigen::Isometry3f& X, 
	    Vector6f& mean, Matrix6f& omega, float& translationalEigenRatio, float& rotationalEigenRatio);

  inline void setImageSize(int r, int c) {_rows = r; _cols = c; _cameraSet=false;}
  inline int imageCols() const { return _cols;}
  inline int imageRows() const { return _rows;}
  inline int scaledImageCols() const { return _refZbuffer.cols();}
  inline int scaledImageRows() const { return _refZbuffer.rows();}


  void setReferenceScene(const Scene* refScene);
  void setCurrentScene(const Scene* currentScene);
  
  inline const Scene* referenceScene() const { return _referenceScene;} 
  inline const Scene* currentScene() const { return _currentScene;} 

  // sets the cloud and conditions the covariances accordingly
  void setReferenceCloud(const PointWithNormalVector*  refPoints, const PointWithNormalSVDVector* refSVDs) __attribute__ ((deprecated)); 
  const PointWithNormalVector* refPoints() const __attribute__ ((deprecated))  {return _refPoints;}
  // sets the cloud and conditions the covariances accordingly
  void setCurrentCloud(const PointWithNormalVector*  currentPoints, const PointWithNormalSVDVector* currentSVDs) __attribute__ ((deprecated)); 
  const PointWithNormalVector* referenceCloud() const __attribute__ ((deprecated)) {return _refPoints;}
  const PointWithNormalSVDVector* referenceSVDs() const __attribute__ ((deprecated)) {return _refSVDs;}
  const PointWithNormalVector* currentCloud() const __attribute__ ((deprecated)) {return _currPoints;}
  const PointWithNormalSVDVector* currentSVDs() const __attribute__ ((deprecated)) {return _currSVDs;}
  const PointWithNormalVector* currPoints() const __attribute__ ((deprecated)) {return _currPoints;}
  
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

  inline float translationalMinEigenRatio()  const {return _translationalMinEigenRatio;}
  inline void setTranslationalMinEigenRatio(float translationalMinEigenRatio_)  {_translationalMinEigenRatio=translationalMinEigenRatio_;}

  inline float rotationalMinEigenRatio()  const {return _rotationalMinEigenRatio;}
  inline void setRotationalMinEigenRatio(float rotationalMinEigenRatio_)  {_rotationalMinEigenRatio=rotationalMinEigenRatio_;}

  inline void setDebug( bool debug_) {_debug = debug_;}
  inline bool debug() const {return _debug;}

  inline int minInliers() const {return _minInliers;}
  inline void setMinInliers(int minInliers_) {_minInliers = minInliers_;}

  inline DepthImage refZBuffer() const {return _refZbuffer;}
  inline DepthImage currZBuffer() const {return _currZbuffer;}

#ifdef _PWN_USE_OPENMP_
  inline int numThreads() const { return _numThreads; }
  inline void setNumThreads(int numThreads_)  { _numThreads = numThreads_; }
#endif //_PWN_USE_OPENMP_


protected:
  // size of the original image
  int _rows,  _cols;

  void _updateCamera();
  void _updateOmegas();

  int _nonLinearUpdate(float& error);
  int _linearUpdate(float& error);

  int _computeStatistics(float& error, Vector6f& mean, Matrix6f& Omega, 
			 float& translationalRatio, float& rotationalRatio, bool onlyFlat = 0) const;

  int _constructLinearSystemQT(Matrix6f& H, Vector6f&b, float& error, bool onlyFlat=false, int numThreads=1, int threadNum = 0 ) const;
  int _constructLinearSystemRT(Matrix12f& H, Vector12f&b, float& error, bool onlyFlat=false, int numThreads=1, int threadNum = 0) const;
  int _constructLinearSystemT(Eigen::Matrix3f& H, Eigen::Vector3f&b, float& error, bool onlyFlat=false, int numThreads=1, int threadNum = 0) const;
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
  
  //parameters to check the quality of the solution
  float _translationalMinEigenRatio;
  float _rotationalMinEigenRatio;


  bool _cameraSet;
  bool _omegasSet;

  const PointWithNormalVector *_refPoints, *_currPoints;
  const PointWithNormalSVDVector *_refSVDs, *_currSVDs;
  const Scene* _referenceScene, *_currentScene;
  Matrix6fVector _currOmegas, _currFlatOmegas;

  DepthImage _refZbuffer, _currZbuffer;
  Eigen::MatrixXi _refIndexImage, _currIndexImage;
  CorrespondenceVector _correspondences;
  int _numCorrespondences;

  Eigen::Isometry3f _T;
  Eigen::Isometry3f _initialT;
  bool _debug;

  // parallelization
  int _numThreads;

};

#endif
