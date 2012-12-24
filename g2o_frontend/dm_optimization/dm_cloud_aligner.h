#ifndef DM_CLOUD_ALIGNER
#define DM_CLOUD_ALIGNER

#include <Eigen/Geometry>
#include "dm_defs.h"

class DMCloudAligner {
 public:
  DMCloudAligner();
  DMCloudAligner(MatrixXus depth0_, MatrixXus depth1_,   
		 int outerIterations_, int innerIterations_,
		 int step_, int minPoints_, int slowAlgorithm_,
		 float r_, float scale_,
		 float curvatureThreshold_, float normalThreshold_,
		 Eigen::Isometry3f initialGuess_, Eigen::Matrix3f cameraMatrix_);
  void setReferenceDepth(MatrixXus depth0_) { _depth0 = depth0_; }
  void setAlignmentDepth(MatrixXus depth1_) { _depth1 = depth1_; }
  float error() { return _error; }
  Eigen::Isometry3f transformation() { return _transformation; } 
  MatrixXus referenceDepth() { return _depth1; }
  MatrixXus alignmentDepth() { return _depth0; }
  Vector6fVector referenceCloud() { return _cloud1; }
  Vector6fVector alignmentCloud() { return _cloud0; }
  CovarianceSVDVector referenceCovariance() { return _svd1; }
  CovarianceSVDVector alignmentCovariance() { return _svd0; }
  CorrespondenceVector correspondence() { return _correspondence; }
  bool alignCloud();

 protected:
  // Parameters
  int _rows, _cols;
  int _outerIterations, _innerIterations;
  int _step, _minPoints, _slowAlgorithm;
  float _r, _scale, _curvatureThreshold, _normalThreshold;
  Eigen::Isometry3f _initialGuess;
  Eigen::Matrix3f _cameraMatrix;
  // Inputs variables.
  MatrixXus _depth0, _depth1;
  // Output variables.
  float _error;
  Eigen::Isometry3f _transformation;
  Vector6fVector _cloud0, _cloud1;
  CovarianceSVDVector _svd0, _svd1;
  CorrespondenceVector _correspondence;
};

#endif
