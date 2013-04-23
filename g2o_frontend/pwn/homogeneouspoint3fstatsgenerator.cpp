#include <Eigen/Eigenvalues>
#include "homogeneouspoint3fstatsgenerator.h"
#include <iostream>
#include <omp.h>

using namespace std;

HomogeneousPoint3fStatsGenerator::HomogeneousPoint3fStatsGenerator(){
  _worldRadius = 0.1;
  _maxImageRadius = 30;
  _minImageRadius = 10;
  _minPoints = 50;
}

void HomogeneousPoint3fStatsGenerator::compute(HomogeneousPoint3fStatsVector& stats, 
					       const HomogeneousPoint3fIntegralImage& integralImage,
					       const Eigen::MatrixXi& intervalImage,
					       const Eigen::MatrixXi& indexImage) {
  assert(integralImage.rows()==intervalImage.rows());
  assert(integralImage.cols()==intervalImage.cols());
  assert(integralImage.rows()==indexImage.rows());
  assert(integralImage.cols()==indexImage.cols());

  #pragma omp parallel for
  for (int c=0; c<indexImage.cols(); ++c){
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
    const int* index = &indexImage.coeffRef(0,c);
    const int* interval = &intervalImage.coeffRef(0,c);
    for (int r=0; r<indexImage.rows(); ++r, ++index, ++interval) {
     
      // is the point valid, is its range valid?
      if (*index<0 || *interval<0)
	continue;
      
      assert(*index<(int)stats.size());
      int imageRadius = *interval;
      if (imageRadius < _minImageRadius)
	imageRadius = _minImageRadius;
      if (imageRadius > _maxImageRadius)
	imageRadius = _maxImageRadius;

      HomogeneousPoint3fAccumulator acc = integralImage.getRegion(r-imageRadius, r+imageRadius, 
								  c-imageRadius, c+imageRadius);
      if (acc.n() < _minPoints)
	continue;
      Eigen::Matrix4f covariance4f = acc.covariance();
      Eigen::Matrix3f covariance3f = covariance4f.block<3,3>(0,0);
      eigenSolver.computeDirect(covariance3f, Eigen::ComputeEigenvectors);
      covariance4f.setZero();
      covariance4f.block<3,3>(0,0) = eigenSolver.eigenvectors();
      covariance4f.block<3,1>(0,3) = eigenSolver.eigenvalues();
      if (covariance4f.coeffRef(0,3) < 0.0f)
	covariance4f.coeffRef(0,3) = 0.0f;
      stats[*index] = covariance4f;
      stats[*index].setN(acc.n());
      stats[*index].setMean(acc.mean());
    }
  }
}

void HomogeneousPoint3fStatsGenerator::compute(HomogeneousNormal3fVector& normals,
					       HomogeneousPoint3fStatsVector& stats,
					       const HomogeneousPoint3fVector& points,
					       const HomogeneousPoint3fIntegralImage& integralImage,
					       const Eigen::MatrixXi& intervalImage,
					       const Eigen::MatrixXi& indexImage,
					       float curvatureThreshold) {
  assert(integralImage.rows()==intervalImage.rows());
  assert(integralImage.cols()==intervalImage.cols());
  assert(integralImage.rows()==indexImage.rows());
  assert(integralImage.cols()==indexImage.cols());

  if(normals.size() != points.size())
    normals.resize(points.size());
  if(stats.size() != points.size())
    stats.resize(points.size());

  #pragma omp parallel for
  for (int c=0; c<indexImage.cols(); ++c){
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
    const int* index = &indexImage.coeffRef(0,c);
    const int* interval = &intervalImage.coeffRef(0,c);
    for (int r=0; r<indexImage.rows(); ++r, ++index, ++interval) {
     
      // is the point valid, is its range valid?
      if (*index<0 || *interval<0)
	continue;
      
      assert(*index<(int)stats.size());
      int imageRadius = *interval;
      if (imageRadius < _minImageRadius)
	imageRadius = _minImageRadius;
      if (imageRadius > _maxImageRadius)
	imageRadius = _maxImageRadius;

      HomogeneousPoint3fAccumulator acc = integralImage.getRegion(r-imageRadius, r+imageRadius, 
								  c-imageRadius, c+imageRadius);
      if (acc.n() < _minPoints)
	continue;
      Eigen::Matrix4f covariance4f = acc.covariance();
      Eigen::Matrix3f covariance3f = covariance4f.block<3, 3>(0, 0);
      eigenSolver.computeDirect(covariance3f, Eigen::ComputeEigenvectors);
      covariance4f.setZero();
      covariance4f.block<3,3>(0,0) = eigenSolver.eigenvectors();
      covariance4f.block<3,1>(0,3) = eigenSolver.eigenvalues();
      if (covariance4f.coeffRef(0,3) < 0.0f)
	covariance4f.coeffRef(0,3) = 0.0f;
      HomogeneousPoint3fStats& stat = stats[*index];
      HomogeneousNormal3f& normal = normals[*index];
      const HomogeneousPoint3f& point = points[*index]; 
      stat = covariance4f;
      stat.setN(acc.n());
      stat.setMean(acc.mean());
      normal = stat.block<4, 1>(0, 0);
      if(stat.curvature() < curvatureThreshold) {
	if(normal.dot(point) > 0)
	  normal = -normal;
      } else {
	normal.setZero();
      }
    }
  }
}
