#include "pointwithnormalaligner.h"
#include <iostream>
#include "g2o/stuff/unscented.h"
#include "g2o/stuff/timeutil.h"
#include <Eigen/Dense>

#ifdef _PWN_USE_OPENMP_
#include <omp.h>
#endif // _PWN_USE_OPENMP_

using namespace Eigen;
using namespace std;
using namespace g2o;

inline Matrix6f jacobian(const Eigen::Isometry3f& X, const Vector6f& p) {
  Matrix6f J = Matrix6f::Zero();
  const Matrix3f& R = X.linear();
  const Vector3f& t = p.head<3>();
  const Vector3f& n = p.tail<3>();
  J.block<3, 3>(0, 0) = R;
  J.block<3, 3>(0, 3) = R * skew(t);
  J.block<3, 3>(3, 3) = R * skew(n);
  return J;
}

PointWithNormalAligner::PointWithNormalAligner() {
  _cameraMatrix <<
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  _rows = 480;
  _cols = 640;
  _updateCamera();
  _lambda = 1e3;
  _scale = 0.5;
  _inlierNormalAngularThreshold = cos(M_PI/6);
  _inlierDistanceThreshold = 3.;
  _inlierMaxChi2 = 9e3;
  _flatCurvatureThreshold = 0.02;
  _flatOmegaDiagonal << 1e3 , 1, 1;
  _outerIterations = 1;
  _nonLinearIterations = 1;
  _linearIterations = 1;
  _minInliers = 100;
  _refPoints = 0;
  _refSVDs = 0;
  _currPoints = 0;
  _currSVDs = 0;
  _numCorrespondences = 0;
  _inlierCurvatureRatioThreshold = 0.2;
  _T = Eigen::Isometry3f::Identity();
  _rotationalMinEigenRatio = 50;
  _translationalMinEigenRatio = 50;
  _debug = false;
  _numThreads = 1;
}


void PointWithNormalAligner::setReferenceCloud(const PointWithNormalVector*  refPoints, const PointWithNormalSVDVector* refSVDs) {
  _refPoints = refPoints;
  _refSVDs = refSVDs;
}
// sets the cloud and conditions tthe covariances accordingly

void PointWithNormalAligner::setCurrentCloud(const PointWithNormalVector*  currentPoints, const PointWithNormalSVDVector* currentSVDs){
  _currPoints = currentPoints;
  _currSVDs = currentSVDs;
  _omegasSet=false;
} 


void PointWithNormalAligner::_updateCamera() {
  if (_cameraSet)
    return;
  _scaledCameraMatrix = _cameraMatrix;
  _scaledCameraMatrix.block<2, 3>(0, 0) *= _scale;
  int r= _scale * (float) _rows ;
  int c= _scale * (float) _cols;
  _refZbuffer.resize(r,c);
  _currZbuffer.resize(r,c);
  _refIndexImage.resize(r,c);
  _currIndexImage.resize(r,c);
  _correspondences.resize(scaledImageCols()*scaledImageRows());
  _numCorrespondences = 0;
  _cameraSet=true;
}

typedef Eigen::DiagonalMatrix<float,3> Diagonal3f;

void PointWithNormalAligner::_updateOmegas() {
  if (_omegasSet)
    return;
  if (! _currPoints || ! _currSVDs) {
    assert (0 && "no point cloud set");
  }
  assert (_currPoints->size() == _currSVDs->size() && "size of currPoints and theit svd should match");
  _currOmegas.resize(_currPoints->size());
  _currFlatOmegas.resize(_currPoints->size());
  float flatKn = 1e2;
  float nonFlatKn = 1.;
  float nonFlatKp = 1.;
  
  Diagonal3f flatOmegaP(_flatOmegaDiagonal.x(), _flatOmegaDiagonal.y(), _flatOmegaDiagonal.z());    
  Diagonal3f flatOmegaN(flatKn, flatKn, flatKn);
  Diagonal3f nonFlatOmegaN(nonFlatKn, nonFlatKn, nonFlatKn);
  Diagonal3f errorFlatOmegaP(1.0, 0.0, 0.0);
  Eigen::Matrix3f inverseCameraMatrix(_cameraMatrix.inverse());
  Eigen::Matrix3f covarianceJacobian(Eigen::Matrix3f::Zero());
    
#ifdef _PWN_USE_OPENMP_
#pragma omp parallel num_threads(_numThreads) 
#endif// _PWN_USE_OPENMP_
  {
      
#ifdef _PWN_USE_OPENMP_
    int threadId = omp_get_thread_num();
#else // _PWN_USE_OPENMP_
    int threadId = 0;
#endif// _PWN_USE_OPENMP_
      
    for (size_t i=threadId; i<_currPoints->size(); i+=_numThreads){
	    const PointWithNormal& point = _currPoints->at(i);
			const PointWithNormalSVD& svd = _currSVDs->at(i);
			_currOmegas[i].setZero();
			if (point.normal().squaredNorm()<1e-3) {
	  		// the point has no normal;
	  		continue;
			}
	
			float curvature = svd.curvature();
			_currFlatOmegas[i].setZero();
			if (curvature<_flatCurvatureThreshold){
	  		_currOmegas[i].block<3,3>(0,0) = svd.U() * flatOmegaP * svd.U().transpose();
	  		_currOmegas[i].block<3,3>(3,3) = flatOmegaN;
	  		_currFlatOmegas[i].block<3,3>(0,0) = svd.U() * errorFlatOmegaP * svd.U().transpose();
	  		_currFlatOmegas[i].block<3,3>(3,3).setIdentity();
			} else {
	  		_currOmegas[i].block<3,3>(0,0) = svd.U() * 
	    																	 Diagonal3f(nonFlatKp/svd.singularValues()(0),
		   	    														 nonFlatKp/svd.singularValues()(1), 
		   	    														 nonFlatKp/svd.singularValues()(2)) * svd.U().transpose();
	  		_currOmegas[i].block<3,3>(3,3) = nonFlatOmegaN;
			}
    }
  }
  _omegasSet=true;
}

int PointWithNormalAligner::align(float& error, Eigen::Isometry3f& X){
  Vector6f mean;
  Matrix6f omega; 
  float translationalEigenRatio;
  float rotationalEigenRatio;
  return align(error, X, mean, omega, translationalEigenRatio, rotationalEigenRatio);
  
}

int PointWithNormalAligner::align(float& error, Eigen::Isometry3f& X, 
				  Vector6f& mean, Matrix6f& omega, float& translationalEigenRatio, float& rotationalEigenRatio){
  _updateOmegas();
  _updateCamera();
  
  // compute the current image and the current imeg
  _T=X.inverse();
  _currPoints->toIndexImage(_currIndexImage, (MatrixXf&)_currZbuffer, _scaledCameraMatrix, Isometry3f::Identity(), 15.0f);
  float squaredThreshold = _inlierDistanceThreshold*_inlierDistanceThreshold;
  int inliers=0;
  double tRemap=0;
  double tCorr = 0;
  double tLinear = 0;
  double tNonLinear = 0;
  double tStats = 0;
  for (int i = 0; i<_outerIterations; i++){
    double tRemapStart = get_time();
    _refPoints->toIndexImage(_refIndexImage, _refZbuffer, _scaledCameraMatrix, _T.inverse(), 15);
    tRemap += get_time() - tRemapStart;
    _numCorrespondences = 0;
    int nGoodPoints = 0;
    int nNoPoint = 0;
    int nNoNormal = 0;
    int nTooDistant = 0;
    int nNormalAngleFail = 0;
    int nCurvatureBad = 0;
    double _tCorr = get_time();
    int threadCorrStartIndex[_numThreads];
    int threadCorrEndIndex[_numThreads];

#ifdef _PWN_USE_OPENMP_
#pragma omp parallel num_threads(_numThreads) 
#endif //_PWN_USE_OPENMP_
    {
      int threadCols = _refIndexImage.cols()/_numThreads;
#ifdef _PWN_USE_OPENMP_
      int threadId = omp_get_thread_num();
#else //_PWN_USE_OPENMP_
      int threadId = 0;
#endif //_PWN_USE_OPENMP_

      int cMin = threadCols * threadId;
      int cMax = (threadId == _numThreads-1) ? _refIndexImage.cols() : threadCols * (threadId+1);
      int corrBlockSize = threadCols * _refIndexImage.rows();
      int _localNGoodPoints = 0;
      int _localNNoPoint = 0;
      int _localNNoNormal = 0;
      int _localNTooDistant = 0;
      int _localNNormalAngleFail = 0;
      int _localNCurvatureBad = 0;

      threadCorrStartIndex[threadId] = threadId*corrBlockSize;
      threadCorrEndIndex[threadId] = threadCorrStartIndex[threadId];

      for (int c=cMin; c<cMax; c++) {
	for (int r=0; r<_refIndexImage.rows(); r++) {
	  int& refIndex = _refIndexImage(r,c);
	  int currIndex = _currIndexImage(r,c);
	  if (refIndex<0 || currIndex<0) {
	    _localNNoPoint++;
	    continue;
	  }
	  // compare euclidean distance and angle for the normals in the remapped frame
	  PointWithNormal pRef=_T*_refPoints->at(refIndex);
	  const PointWithNormal& pCurr =_currPoints->at(currIndex);
	  if (pRef.normal().squaredNorm()==0 || pCurr.normal().squaredNorm()==0){
	    refIndex = -refIndex;
	    _localNNoNormal++;
	    continue;
	  }
	
	  Vector3f dp =pCurr.point()-pRef.point();
	  if (dp.squaredNorm()>squaredThreshold){
	    refIndex = -refIndex;
	    _localNTooDistant ++;
	    continue;
	  }

	  if (pCurr.normal().dot(pRef.normal()) < _inlierNormalAngularThreshold) {
	    refIndex = -refIndex;
	    _localNNormalAngleFail++;
	    continue;
	  }

	  const PointWithNormalSVD& refSVD = _refSVDs->at(refIndex);
	  const PointWithNormalSVD& currSVD = _currSVDs->at(currIndex);
	  float refCurvature = refSVD.curvature();
	  float currCurvature = currSVD.curvature();
	  if (refCurvature < _flatCurvatureThreshold)
	    refCurvature = _flatCurvatureThreshold;
	  if (currCurvature < _flatCurvatureThreshold)
	    currCurvature = _flatCurvatureThreshold;

	  float logRatio = log(refCurvature +1e-5) - log(currCurvature + 1e-5);
	  if (fabs(logRatio)>_inlierCurvatureRatioThreshold){
	    refIndex = -refIndex;
	    _localNCurvatureBad++;
	    continue;
	  }
	  _localNGoodPoints++;
	  int& corrIndex=threadCorrEndIndex[threadId];
	  _correspondences[corrIndex].i1=refIndex;
	  _correspondences[corrIndex].i2=currIndex;
	  corrIndex ++;
	}
      }
      nGoodPoints += _localNGoodPoints;
      nNoPoint += _localNNoPoint;
      nNoNormal += _localNNoNormal;
      nTooDistant += _localNTooDistant;
      nNormalAngleFail += _localNNormalAngleFail;
      nCurvatureBad += _localNCurvatureBad;
    }
    // now pack the resulting correspondences
    for(int t=0; t<_numThreads; t++){
      int cstart = threadCorrStartIndex[t];
      int cend = threadCorrEndIndex[t];
      for (int k = cstart; k<cend; k++, _numCorrespondences++){
	_correspondences[_numCorrespondences] = _correspondences[k];
      }
    }
    tCorr += get_time() - _tCorr;
    for (size_t k=_numCorrespondences; k<_correspondences.size(); k++){
      _correspondences[_numCorrespondences].i1=-1;
      _correspondences[_numCorrespondences].i2=-1;
    }
    if (_debug) {
      cerr << "goodPoints: " << nGoodPoints << endl;
      cerr << "noPoint: " << nNoPoint << endl;
      cerr << "noNormal: " << nNoNormal << endl;
      cerr << "tooDistant: " << nTooDistant << endl;
      cerr << "badAngle: " << nNormalAngleFail << endl;
      cerr << "badCurvature: " << nCurvatureBad << endl;
    }
    if (_numCorrespondences<_minInliers){
      _numCorrespondences = 0;
      return -1;
    }
    double _tLinear = get_time();
    float error;
    for (int li=0; li<_linearIterations; li++){
      inliers = _linearUpdate(error);
      if (inliers < _minInliers) {
	return -2;
      }
    }
    
    tLinear += get_time() - _tLinear;
    double _tNonLinear = get_time();
    for (int nli=0; nli<_nonLinearIterations; nli++){
      int inliers = _nonLinearUpdate(error);
      if (inliers < _minInliers){
	return -3;
      }
    }

    tNonLinear += get_time() - _tNonLinear;
  }
  tStats = get_time();
  inliers = _computeStatistics(error, mean, omega, translationalEigenRatio, rotationalEigenRatio, false);
  tStats = get_time() - tStats;
  
  cerr << "*** TIMINGS ***" << endl;
  cerr << "tRemap: " << tRemap << endl;
  cerr << "tCorr: " << tCorr << endl;
  cerr << "tLinear: " << tLinear << endl;
  cerr << "tNonLinear: " << tNonLinear << endl;
  cerr << "tStats: " << tStats << endl;

  if (rotationalEigenRatio > _rotationalMinEigenRatio || translationalEigenRatio > _translationalMinEigenRatio) {
    cerr << "************** WARNING SOLUTION MIGHT BE INVALID (eigenratio failure) **************" << endl;
    cerr << "tr: " << translationalEigenRatio << " rr: " << rotationalEigenRatio << endl;
    cerr << "************************************************************************************" << endl;
  } else {
    cerr << "************** I FOUND SOLUTION VALID SOLUTION   (eigenratio ok) *******************" << endl;
    cerr << "tr: " << translationalEigenRatio << " rr: " << rotationalEigenRatio << endl;
    cerr << "************************************************************************************" << endl;
  }

  if (_debug) {
    cerr << "Solution statistics in (t, mq)" << endl;
    cerr << "mean: " << mean.transpose() << endl;
    cerr << "Omega: " << endl;
    cerr << omega << endl;
  }

  if (inliers < _minInliers)
    return -4;
    
  X = _T.inverse();
  return inliers;
}
  
int PointWithNormalAligner::_constructLinearSystemQT(Matrix6f& H, Vector6f&b, float& error, bool onlyFlat, int numThreads, int threadNum) const{
  b = Vector6f::Zero();
  H = Matrix6f::Zero();
  error = 0;
  int numInliers = 0;
  // cerr << "correspondence.size(): " << _correspondences.size() << endl;
  // cerr << "refPoints.size(): " << _refPoints->size() << endl;
  // cerr << "currPoints.size(): " << _currPoints->size() << endl;
  // cerr << "omegas.size(): " << _currOmegas.size() << endl;
  const Matrix6fVector& omegas = (onlyFlat)? _currFlatOmegas : _currOmegas;
    
  int n = _numCorrespondences/numThreads;
  int imin = threadNum *n;
  int imax = (threadNum == numThreads -1) ? _numCorrespondences : (threadNum+1) *n;
  for(int i = imin; i < imax; i++){
    //cerr << "corr[" << i << "]: ";
    const Correspondence& corr = _correspondences[i];
    const PointWithNormal& pref  = _refPoints->at(corr.i1);
    const PointWithNormal& pcurr = _currPoints->at(corr.i2);
    //cerr << corr.i1 << " " << corr.i2 << endl;
    const Matrix6f& omega = omegas.at(corr.i2);
    if (omega.squaredNorm()==0)
      continue;
    	
    Vector6f e = _T*pref - pcurr;
      
    float localError = e.transpose() * omega * e;
    if(localError > _inlierMaxChi2)
      continue;
    numInliers++;
    error += localError;
    Matrix6f J = jacobian(_T, pcurr);
    b += J.transpose() * omega * e;
    H += J.transpose() * omega * J;
  }
  return numInliers;
}

int PointWithNormalAligner::_constructLinearSystemRT(Matrix12f& H, Vector12f&b, float& error, bool onlyFlat, int numThreads, int threadNum) const{
  b.setZero();
  H.setZero();
  error = 0;
  int numInliers = 0;
  Matrix6x12f J;
  const Matrix6fVector& omegas = (onlyFlat)? _currFlatOmegas : _currOmegas;

  int n = _numCorrespondences/numThreads;
  int imin = threadNum *n;
  int imax = (threadNum == numThreads -1) ? _numCorrespondences : (threadNum+1) *n;
  for(int i = imin; i < imax; i++){
    const Correspondence& corr = _correspondences[i];
    const PointWithNormal& pref  = _refPoints->at(corr.i1);
    const PointWithNormal& pcurr = _currPoints->at(corr.i2);
    const Matrix6f& omega = omegas.at(corr.i2);
    if (omega.squaredNorm()==0)
      continue;
    
    Vector6f e = _T*pref - pcurr;
      
    float localError = e.transpose() * omega * e;
    if(localError > _inlierMaxChi2)
      continue;
    numInliers++;
    error += localError;
    J.setZero();
    Vector3f t=pref.block<3,1>(0,0);
    Vector3f n=pref.block<3,1>(3,0);
    J.block<1,3>(0,0)=t.transpose();
    J.block<1,3>(1,3)=J.block<1,3>(0,0);
    J.block<1,3>(2,6)=J.block<1,3>(0,0);
    J.block<3,3>(0,9)=Matrix3f::Identity();
    J.block<1,3>(3,0)=n.transpose();
    J.block<1,3>(4,3)=J.block<1,3>(3,0);
    J.block<1,3>(5,6)=J.block<1,3>(3,0);
    
    b += -J.transpose() * omega * e;
    H += J.transpose() * omega * J;
  }
  return numInliers;
}

int PointWithNormalAligner::_constructLinearSystemT(Matrix3f& H, Vector3f&b, float& error, bool onlyFlat, int numThreads, int threadNum) const{
  b.setZero();
  H.setZero();
  error = 0;
  int numInliers = 0;
  Matrix6x12f J;
  const Matrix6fVector& omegas = (onlyFlat)? _currFlatOmegas : _currOmegas;


  int n = _numCorrespondences/numThreads;
  int imin = threadNum *n;
  int imax = (threadNum == numThreads -1) ? _numCorrespondences : (threadNum+1) *n;
  for(int i = imin; i < imax; i++){
    const Correspondence& corr = _correspondences[i];
    const PointWithNormal& pref  = _refPoints->at(corr.i1);
    const PointWithNormal& pcurr = _currPoints->at(corr.i2);
    const Matrix6f& omega = omegas.at(corr.i2);
    if (omega.squaredNorm()==0)
      continue;
    
    Vector6f e = _T*pref - pcurr;
    float localError = e.transpose() * omega * e;
    if(localError > _inlierMaxChi2)
      continue;
    numInliers++;
    error += localError;
    b += -omega.block<3,3>(0,0) * e.head<3>();
    H += omega.block<3,3>(0,0);
  }
  return numInliers;
}

int PointWithNormalAligner::_computeStatistics(float& error, Vector6f& mean, Matrix6f& Omega, 
					       float& translationalRatio, float& rotationalRatio,
					       bool onlyFlat) const{
  typedef g2o::SigmaPoint<Vector6f> MySigmaPoint;
  typedef std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > MySigmaPointVector;

  
  Vector6f b;
  Matrix6f H;
  b.setZero();
  H.setZero();

  translationalRatio = std::numeric_limits<float>::max();
  rotationalRatio = std::numeric_limits<float>::max();

  error = 0;
  int inliers = 0;
#ifdef _PWN_USE_OPENMP_
#pragma omp parallel num_threads(_numThreads) 
#endif //_PWN_USE_OPENMP_
  {
    float tempError;
    Vector6f tempb;
    Matrix6f tempH;
    
#ifdef _PWN_USE_OPENMP_
    int threadId = omp_get_thread_num();
#else
    int threadId = 0;
#endif //_PWN_USE_OPENMP_

    int tempInliers = _constructLinearSystemQT(tempH,tempb,tempError, onlyFlat, _numThreads, threadId);
#ifdef _PWN_USE_OPENMP_
#pragma omp critical 
#endif //_PWN_USE_OPENMP_
    {
      inliers += tempInliers;
      error += tempError;
      b+= tempb;
      H+= tempH;
    }
  }

  if(inliers < _minInliers){
    return inliers;
  }

  JacobiSVD<Matrix6f> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // if (_debug)
  //   cerr << "singularValues = " << svd.singularValues().transpose() << endl;
  // //TODO: check the ratio of the singular values to see if the solution makes sense
  // float ratio = svd.singularValues()(0)/svd.singularValues()(5);
  // if (_debug)
  //   cerr << "singularValuesRatio = " << ratio << endl;

  Matrix6f localSigma = svd.solve(Matrix6f::Identity());
  MySigmaPointVector sigmaPoints;
  Vector6f localMean = Vector6f::Zero();
  g2o::sampleUnscented(sigmaPoints, localMean, localSigma);
  
  // remap each of the sigma points to their original position
  for(size_t i=0; i<sigmaPoints.size(); i++){
    MySigmaPoint& p = sigmaPoints[i];
    p._sample = t2v(_T*v2t(p._sample));
  }
  
  // reconstruct the gaussian 
  g2o::reconstructGaussian(mean,localSigma, sigmaPoints);

  // compute the information matrix from the covariance
  Omega = localSigma.inverse();
  
  // have a look at the svd of the rotational and the translational part;
  JacobiSVD<Matrix3f> partialSVD;
  partialSVD.compute(Omega.block<3,3>(0,0));
  translationalRatio = partialSVD.singularValues()(0)/partialSVD.singularValues()(2);
  if (_debug){
    cerr << "singular values of the Omega_t: " << partialSVD.singularValues().transpose() << endl;
    cerr << "translational_ratio: " <<  translationalRatio << endl;
  } 
  partialSVD.compute(Omega.block<3,3>(3,3));
  rotationalRatio = partialSVD.singularValues()(0)/partialSVD.singularValues()(2);
  if (_debug) {
    cerr << "singular values of the Omega_r: " << partialSVD.singularValues().transpose() << endl; 
    cerr << "rotational_ratio: " << rotationalRatio << endl;
  }
  return inliers;
}

int PointWithNormalAligner::_nonLinearUpdate(float& error){
  Vector6f b;
  b.setZero();
  Matrix6f H;
  H.setZero();

  error = 0;
  int inliers = 0;
#ifdef _PWN_USE_OPENMP_
#pragma omp parallel num_threads(_numThreads)
#endif //_PWN_USE_OPENMP_
  {
    float tempError;
    Vector6f tempb;
    Matrix6f tempH;

#ifdef _PWN_USE_OPENMP_
    int threadId = omp_get_thread_num();
#else
    int threadId = 0;
#endif //_PWN_USE_OPENMP_

    int tempInliers = _constructLinearSystemQT(tempH,tempb,tempError, false, _numThreads, threadId);
#ifdef _PWN_USE_OPENMP_
#pragma omp critical 
#endif //_PWN_USE_OPENMP_
    {
      inliers += tempInliers;
      error += tempError;
      b+= tempb;
      H+= tempH;
    }
  }

  if(inliers < _minInliers){
    return inliers;
  }
  if (_debug) {
    cerr << "NonLinear! initial error: " << error << " initial inliers:" << inliers << endl;
  }
  Matrix6f H2 = H +Matrix6f::Identity()*_lambda;
  Vector6f dx = H2.ldlt().solve(-b);
  _T  = _T*v2t(dx);
  return inliers;
}

int PointWithNormalAligner::_linearUpdate(float& error){
  Vector12f bRt;
  Matrix12f HRt;
    
  bRt.setZero();
  HRt.setZero();
  error = 0;
  int inliers = 0;
#ifdef _PWN_USE_OPENMP_
#pragma omp parallel num_threads(_numThreads) 
#endif //_PWN_USE_OPENMP_
  {
    float tempError;
    Vector12f tempB;
    Matrix12f tempH;
#ifdef _PWN_USE_OPENMP_
    int threadId = omp_get_thread_num();
#else
    int threadId = 0;
#endif //_PWN_USE_OPENMP_

    int tempInliers = _constructLinearSystemRT(tempH, tempB,tempError, false, _numThreads, threadId);
#ifdef _PWN_USE_OPENMP_
#pragma omp critical 
#endif //_PWN_USE_OPENMP_
    {
      inliers += tempInliers;
      error += tempError;
      bRt+= tempB;
      HRt+= tempH;
    }
  }

  if (_debug){
    cerr << "Linear! initial error: " << error << " initial inliers:" << inliers << endl;
  }
  if(inliers < _minInliers){
    return inliers;
  }

  Vector12f x=homogeneous2vector(_T.matrix());
  HRt+=Matrix12f::Identity()*_lambda;
  LDLT<Matrix12f> ldlt(HRt);
  if (ldlt.isNegative())
    return 0;
  x=ldlt.solve(bRt); // using a LDLT factorizationldlt;
  Matrix4f _X = _T.matrix()+vector2homogeneous(x);
    
  // recondition the rotation 
  JacobiSVD<Matrix3f> svd(_X.block<3,3>(0,0), Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (_debug) {
    cerr << "singular values: " << svd.singularValues().transpose() << endl;
  }
  if (svd.singularValues()(0)<.5)
    return false;

  Matrix3f R=svd.matrixU()*svd.matrixV().transpose();
  _T.linear()=R;
  _T.translation() = _X.block<3,1>(0,3);

  // recompute the translation
  Matrix3f Ht;
  Ht.setZero();
  Vector3f bt;
  bt.setZero();
  error = 0;
  inliers = 0;
#ifdef _PWN_USE_OPENMP_
#pragma omp parallel num_threads(_numThreads) 
#endif //_PWN_USE_OPENMP_
  {
    float tempError;
    Vector3f tempB;
    Matrix3f tempH;
#ifdef _PWN_USE_OPENMP_
    int threadId = omp_get_thread_num();
#else
    int threadId = 0;
#endif //_PWN_USE_OPENMP_

    int tempInliers =  _constructLinearSystemT(tempH, tempB,tempError, false, _numThreads, threadId);
#ifdef _PWN_USE_OPENMP_
#pragma omp critical 
#endif //_PWN_USE_OPENMP_
    {
      inliers += tempInliers;
      error += tempError;
      bt+= tempB;
      Ht+= tempH;
    }
  }

  if (_debug) {
    cerr << "Linear! errorAfterRot: " << error << " inliersAfterRot:" << inliers << endl;
  }
  if(inliers < _minInliers){
    return inliers;
  }

  Vector3f dt;
  Ht+=Matrix3f::Identity()*_lambda;
  dt = Ht.ldlt().solve(bt);
  _T.translation()+=dt;


  return inliers;
}

