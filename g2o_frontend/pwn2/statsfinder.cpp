#include "statsfinder.h"
#include <Eigen/Eigenvalues>
#include <omp.h>

#include <iostream>

namespace pwn {

StatsFinder::StatsFinder(){
  _worldRadius = 0.1;
  _maxImageRadius = 30;
  _minImageRadius = 10;
  _minPoints = 50;
}

void StatsFinder::compute(PointStatsVector &stats,
                           const PointIntegralImage &integralImage,
					       const Eigen::MatrixXi &intervalImage,
					       const Eigen::MatrixXi &indexImage) {
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

      PointAccumulator acc = integralImage.getRegion(r-imageRadius, r+imageRadius,
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

void StatsFinder::compute(NormalVector& normals,
                           PointStatsVector& stats,
                           const PointVector& points,
                           const PointIntegralImage& integralImage,
					       const Eigen::MatrixXi& intervalImage,
					       const Eigen::MatrixXi& indexImage,
					       float curvatureThreshold) {
  assert(integralImage.rows() == intervalImage.rows());
  assert(integralImage.cols() == intervalImage.cols());
  assert(integralImage.rows() == indexImage.rows());
  assert(integralImage.cols() == indexImage.cols());

  if(stats.size() != points.size())
    stats.resize(points.size());
  if(normals.size() != points.size())
    normals.resize(points.size());
  
  Normal zero;
  zero.setZero();  
  std::fill(stats.begin(), stats.end(), PointStats());
  std::fill(normals.begin(), normals.end(), zero);

#pragma omp parallel for
  for (int c=0; c<indexImage.cols(); ++c) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
    const int *index = &indexImage.coeffRef(0,c);
    const int *interval = &intervalImage.coeffRef(0,c);
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

      const PointAccumulator acc = integralImage.getRegion(r-imageRadius, r+imageRadius,
								  c-imageRadius, c+imageRadius);
      if (acc.n() < _minPoints)
	continue;
      Eigen::Matrix4f covariance4f = acc.covariance();
      Eigen::Matrix3f covariance3f = covariance4f.block<3, 3>(0, 0);
      eigenSolver.computeDirect(covariance3f, Eigen::ComputeEigenvectors);
      covariance4f.setZero();
      covariance4f.block<3, 3>(0, 0) = eigenSolver.eigenvectors();
      covariance4f.block<3, 1>(0, 3) = eigenSolver.eigenvalues();
      if (covariance4f(0, 3) < 0.0f)
	covariance4f(0, 3) = 0.0f;
      PointStats &stat = stats[*index];
      Normal &normal = normals[*index];
      const Point &point = points[*index];
      stat = covariance4f;
      stat.setN(acc.n());
      stat.setMean(acc.mean());
      normal.block<4, 1>(0, 0) = stat.block<4, 1>(0, 0);
      if(stat.curvature() < curvatureThreshold) {
	if(normal.dot(point) > 0)
	  normal = -normal;
      } else {
	normal.setZero();
      }    
    }
  }
}

}
