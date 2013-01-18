#include "pointwithnormalmerger.h"
#include "pixelmapper.h"
#include "pointwithnormalaligner.h"
#include <iostream> 

using namespace Eigen;

PointWithNormalMerger::PointWithNormalMerger() {
  _alpha = 0.1f;
  _baseLine = 0.075f;
  _cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  _indexImage.fill(-1);
  _depthImage.fill(std::numeric_limits<float>::max());
  _points.clear();
  _mergedPoints.clear();
  _covariancesAccumulator.clear();
  //_covariancesSVDsVector.clear();
}

void PointWithNormalMerger::addCloud(Eigen::Isometry3f t, const PointWithNormalVector points_) {
  PointWithNormalVector transformedPoints = t * points_;
  _points.insert(_points.end(), transformedPoints.begin(), transformedPoints.end());
}

void PointWithNormalMerger::computeAccumulator() {
  // Compute skin.
  //_depthImage = MatrixXf(480, 640);
  _indexImage.resize(480, 640);
  _points.toIndexImage(_indexImage, _depthImage, _cameraMatrix, Isometry3f::Identity());
  
  // Compute sigma for each point and create image accumulator.
  PixelMapper pm;
  pm.setCameraMatrix(_cameraMatrix);
  pm.setTransform(Isometry3f::Identity());
  Matrix3f inverseCameraMatrix = _cameraMatrix.inverse();
  float fB = (_baseLine * _cameraMatrix(0, 0)); // kinect baseline * focal lenght;
  CovarianceAccumulator covAcc; 
  _covariancesAccumulator.resize(_points.size());
  fill(_covariancesAccumulator.begin(), _covariancesAccumulator.end(), covAcc);
  //_covariancesSVDsVector.resize(_points.size());
  //SelfAdjointEigenSolver<Matrix3f> eigenSolver;
  for(size_t i = 0; i < _points.size(); i++ ) {
    PointWithNormal &p = _points[i];
    Vector2i coord = pm.imageCoords(pm.projectPoint(p.point()));
    if(p.point() == Vector3f::Zero() || p.normal() == Vector3f::Zero())
      continue;
    if(coord[0] < 0 || coord[0] > _depthImage.cols())
      continue;
    if(coord[1] < 0 || coord[1] > _depthImage.rows())
      continue;
    int index = _indexImage(coord[1], coord[0]);
    float skinZ = _depthImage(coord[1], coord[0]);
    if(_covariancesAccumulator[index]._omegaAcc == Matrix3f::Zero()) {
      float skinZVariation = (_alpha*skinZ*skinZ)/(fB+skinZ*_alpha);
      skinZVariation *= skinZVariation;
      Diagonal3f skinImageCovariance(1.0f, 1.0f, skinZVariation);
      Matrix3f skinCovarianceJacobian;
      skinCovarianceJacobian <<
	skinZ, 0, (float)coord[0],
	0, skinZ, (float)coord[1],
	0, 0, 1;
      skinCovarianceJacobian = inverseCameraMatrix*skinCovarianceJacobian;
      Matrix3f skinWorldCovariance = skinCovarianceJacobian * skinImageCovariance * skinCovarianceJacobian.transpose();
      _covariancesAccumulator[index]._omegaAcc += skinWorldCovariance.inverse();
      _covariancesAccumulator[index]._pointsAcc += skinWorldCovariance.inverse()*p.point();
    }
    Vector3f currentNormal = p.normal();
    Vector3f skinNormal = _points[index].normal();
    float z = p[2];
    if(abs(z - skinZ) > 0.05)
      continue;
    if(acosf(skinNormal.dot(currentNormal)) > M_PI/4.0f)
      continue;
    float zVariation = (_alpha*z*z)/(fB+z*_alpha);
    zVariation *= zVariation;
    Diagonal3f imageCovariance(1.0f, 1.0f, zVariation);
    Matrix3f covarianceJacobian;
    covarianceJacobian <<
      z, 0, (float)coord[0],
      0, z, (float)coord[1],
      0, 0, 1;
    covarianceJacobian = inverseCameraMatrix*covarianceJacobian;
    Matrix3f worldCovariance = covarianceJacobian * imageCovariance * covarianceJacobian.transpose();
    _covariancesAccumulator[index]._omegaAcc += worldCovariance.inverse();
    _covariancesAccumulator[index]._pointsAcc += worldCovariance.inverse()*p.point();
    /*_pointsAccumulator[i]._omegaAcc = worldCovariance;
    PointWithNormalSVD& svd = _covariancesSVDsVector[i];
    svd._mean = p.point();
    svd._n = 1;
    eigenSolver.compute(worldCovariance);
    svd._U = eigenSolver.eigenvectors();
    svd._singularValues = eigenSolver.eigenvalues();
    if(svd._singularValues(0) < 0)
      svd._singularValues(0) = 0;
      svd.updateCurvature();*/
  }
}

void PointWithNormalMerger::extractMergedCloud() {
  _mergedPoints.resize(_covariancesAccumulator.size());
  Vector3f pointMean;
  for(size_t i = 0; i < _covariancesAccumulator.size(); i++) {
    if(_covariancesAccumulator[i]._omegaAcc == Matrix3f::Zero()) {
      _mergedPoints[i].setPoint(Vector3f::Zero());
      _mergedPoints[i].setNormal(Vector3f::Zero());
      continue;
    }
    pointMean = _covariancesAccumulator[i]._omegaAcc.inverse()*_covariancesAccumulator[i]._pointsAcc;
    _mergedPoints[i].setPoint(pointMean);
    _mergedPoints[i].setNormal(Vector3f::Zero());
  }
}
