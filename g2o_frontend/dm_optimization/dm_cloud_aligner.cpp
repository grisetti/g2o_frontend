#include "dm_cloud_aligner.h"
#include "dm_normals.h"
#include "dm_cloud.h"
#include "dm_solve.h"
#include "dm_utils.h"
#include <iostream>

using namespace std;
using namespace Eigen;

DMCloudAligner::DMCloudAligner() {
  _depth0 = MatrixXus();
  _depth1 = MatrixXus();
  _rows = 0;
  _cols = 0;
  _outerIterations = 10;
  _innerIterations = 10;
  _step = 1;
  _minPoints = 50;
  _slowAlgorithm = 0;
  _r = 0.1;
  _scale = 0.25;
  _curvatureThreshold = 0.02;
  _normalThreshold = M_PI/6;
  _initialGuess = Isometry3f::Identity();
  _cameraMatrix = Matrix3f::Zero();
  _error = std::numeric_limits<float>::max();
  _transformation = Isometry3f::Identity();
  _cloud0 = Vector6fVector();
  _cloud1 = Vector6fVector();
  _svd0 = CovarianceSVDVector();
  _svd1 = CovarianceSVDVector();
  _correspondence = CorrespondenceVector();
}

DMCloudAligner::DMCloudAligner(MatrixXus depth0_, MatrixXus depth1_,   
			       int outerIterations_, int innerIterations_,
			       int step_, int minPoints_, int slowAlgorithm_,
			       float r_, float scale_,
			       float curvatureThreshold_, float normalThreshold_,
			       Isometry3f initialGuess_, Matrix3f cameraMatrix_) {
  _depth0 = depth0_;
  _depth1 = depth1_;
  _rows = _depth1.rows();
  _cols = _depth1.cols();
  _outerIterations = outerIterations_; 
  _innerIterations = innerIterations_;
  _step = step_;
  _minPoints = minPoints_;
  _slowAlgorithm = slowAlgorithm_;
  _r = r_;
  _scale = scale_;
  _curvatureThreshold = curvatureThreshold_; 
  _normalThreshold = normalThreshold_;
  _initialGuess = initialGuess_;
  _cameraMatrix = cameraMatrix_;
  _error = std::numeric_limits<float>::max();
  _transformation = Isometry3f::Identity();
  _cloud0.reserve(_depth0.rows() * _depth0.cols());
  _cloud1.reserve(_depth1.rows() * _depth1.cols());
  _svd0.reserve(_depth0.rows() * _depth0.cols());
  _svd1.reserve(_depth1.rows() * _depth1.cols());
  _correspondence.reserve(_depth1.rows() * _depth1.cols());
}

bool DMCloudAligner::alignCloud() {
  clock_t begin = getMilliSecs();
  _transformation = Isometry3f::Identity();
  
  _rows = _depth1.rows();
  _cols = _depth1.cols();

  // Cast images to float type.
  cerr << "Converting input images to float type...";
  MatrixXf depth0f(_rows, _cols), depth1f(_rows, _cols);
  img2depth(depth0f, _depth0);
  img2depth(depth1f, _depth1);
  cerr << " done !" << endl;

  // Create 3D point clouds with normals.
  cerr << "Creating 3D point cloud...";
  depth2cloud(_cloud0, depth0f, _cameraMatrix);
  depth2cloud(_cloud1, depth1f, _cameraMatrix);
  cerr << " done !" << endl;

  // Create matrices of pointers from the vectors.
  MatrixXf curvature0(_rows, _cols), curvature1(_rows, _cols);
  Vector6fPtrMatrix cloud0Ptr(_rows, _cols), cloud1Ptr(_rows, _cols);
  CovarianceSVDPtrMatrix svd0Ptr(_rows, _cols), svd1Ptr(_rows, _cols);
  _svd0 = CovarianceSVDVector(_cloud0.size()); 
  _svd1 = CovarianceSVDVector(_cloud1.size());
  Matrix6fVector omega0, omega1;
  Matrix6fPtrMatrix omega0Ptr, omega1Ptr;  
  MatrixXf zBuffer(_rows, _cols);
  cloud0Ptr.fill(0);
  svd0Ptr.fill(0);
  omega0Ptr.fill(0);
  cloud1Ptr.fill(0);
  svd1Ptr.fill(0);
  omega1Ptr.fill(0);
  cloud2mat(cloud0Ptr, omega0Ptr, svd0Ptr,
            _cloud0, omega0, _svd0,
            Isometry3f::Identity(), _cameraMatrix, zBuffer);
  cloud2mat(cloud1Ptr, omega1Ptr, svd1Ptr,
            _cloud1, omega1, _svd1,
            Isometry3f::Identity(), _cameraMatrix, zBuffer);
  
  // Compute normals.
  cerr << "Computing normals... ";
  computeNormals(cloud0Ptr, curvature0, svd0Ptr, _cameraMatrix, _r, _slowAlgorithm, _step, _minPoints);
  computeNormals(cloud1Ptr, curvature1, svd1Ptr, _cameraMatrix, _r, _slowAlgorithm, _step, _minPoints);
  cerr << " done !" << endl;

  // Compute omegas.
  cerr << "Computing omegas... ";
  svd2omega(omega0, _svd0);
  svd2omega(omega1, _svd1);
  cerr << " done !" << endl;
  
  // Scale factors.
  Matrix3f cameraMatrixScaled = _cameraMatrix;
  cameraMatrixScaled.block<2, 3>(0, 0) *= _scale;
  int _r = ((float)_rows * _scale);
  int _c = ((float)_cols * _scale);

  // Create scaled clouds variables.
  Vector6fPtrMatrix cloud0PtrScaled(_r, _c), cloud1PtrScaled(_r, _c);
  Matrix6fPtrMatrix omega0PtrScaled(_r, _c), omega1PtrScaled(_r, _c);
  CovarianceSVDPtrMatrix svd0PtrScaled(_r, _c), svd1PtrScaled(_r, _c);
  Vector6fPtrMatrix corrP0(_r,_c), corrP1(_r,_c);
  Matrix6fPtrMatrix corrOmega(_r, _c);
  
  omega0PtrScaled.fill(0);
  cloud0PtrScaled.fill(0);
  svd0PtrScaled.fill(0);
  omega1PtrScaled.fill(0);
  cloud1PtrScaled.fill(0);
  svd1PtrScaled.fill(0);
  // Scale cloud1 and update it with the initial guess.
  cloud2mat(cloud0PtrScaled, omega0PtrScaled, svd0PtrScaled,
	    _cloud0, omega0, _svd0,
	    _initialGuess.inverse(), cameraMatrixScaled, zBuffer);
  
  // Scale cloud0.
  cloud2mat(cloud1PtrScaled, omega1PtrScaled, svd1PtrScaled,
	    _cloud1, omega1, _svd1,
	    Isometry3f::Identity(), cameraMatrixScaled, zBuffer);

  for (int k = 0; k < _outerIterations; k++) {
    clock_t oStart = getMilliSecs();
    
    // Compute correspondences.
    corrP0.fill(0);
    corrP1.fill(0);
    corrOmega.fill(0);
    _correspondence.clear();
    int corrFound = 0;
    for (int i = 0; i < cloud1PtrScaled.cols(); i++) {
      for (int j = 0; j < cloud1PtrScaled.rows(); j++) {
	if (! cloud0PtrScaled(j, i) || !cloud1PtrScaled(j, i))
	  continue;
	Vector6f& p0 = *(cloud0PtrScaled(j, i));
	Vector6f& p1 = *(cloud1PtrScaled(j, i));
	if (p0.tail<3>().squaredNorm() <= 1e-3 || p1.tail<3>().squaredNorm() <= 1e-3)
	  continue;
	SVDMatrix3f& svd0 = *svd0PtrScaled(j, i);
	SVDMatrix3f& svd1 = *svd1PtrScaled(j, i);
	float c0 = svd0.curvature();
	float c1 = svd1.curvature();
	if (c0 > _curvatureThreshold || c1 > _curvatureThreshold)
	  continue;
	Vector3f n0 = p0.tail<3>();
	Vector3f n1 = p1.tail<3>();
	if (n0.dot(n1) < _normalThreshold)
	  continue;
	_correspondence.push_back(Correspondence(cloud0PtrScaled(j, i), cloud1PtrScaled(j, i)));	
	corrP0(j, i) = &p0;
	corrP1(j, i) = &p1;
	corrOmega(j, i) = omega0PtrScaled(j, i);
	corrFound ++;
      }
    }
    cerr << "found " << corrFound << " correspondences" << endl;
    
    // Compute transformation between the two clouds.
    Isometry3f result = Isometry3f::Identity();
    int size = _r * _c;  
    int inliers = 0; 
    clock_t start = getMilliSecs();
    for (int i = 0; i < _innerIterations; i++) {
      inliers = pwn_iteration(_error, result,
			      corrP0.data(), corrP1.data(), corrOmega.data(),
			      size, _transformation, 
			      numeric_limits<float>::max(), 0);
    }
    if(inliers < 50)
      return false;
    cout << "k: " << k << " " << inliers << " " << _error << " " << endl;
    cout << "Time optimization : " << getMilliSecs() - start << " ms" << endl;
    cout << "Time global iteration: " << getMilliSecs() - oStart << " ms" << endl;
    cout << "---------------------------------------------------------------" << endl;  
    _transformation = result;

    // Update alignment cloud.
    omega0PtrScaled.fill(0);
    cloud0PtrScaled.fill(0);
    svd0PtrScaled.fill(0);
    cloud2mat(cloud0PtrScaled, omega0PtrScaled, svd0PtrScaled,
	      _cloud0, omega0, _svd0,
	      _transformation.inverse(), cameraMatrixScaled, zBuffer);
  }
  cout << "Time total computation: " << getMilliSecs() - begin << " ms" << endl;
  cout << "Final transformation computed: " << endl << _transformation.matrix() << endl;
  return true;
}
