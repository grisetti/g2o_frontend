#include "statscalculator.h"
#include <Eigen/Eigenvalues>
#include <omp.h>

#include <iostream>

namespace pwn {

StatsCalculator::StatsCalculator(){
  _worldRadius = 0.1;
  _maxImageRadius = 30;
  _minImageRadius = 10;
  _minPoints = 50;
}

void StatsCalculator::compute(PointStatsVector &stats,
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
      PointStats& stat = stats[*index];
      Eigen::Matrix3f covariance3f = acc.covariance().block<3,3>(0,0);
      eigenSolver.computeDirect(covariance3f, Eigen::ComputeEigenvectors);
      Eigen::Vector4f eigenvalues;
      if (eigenvalues(0) < 0.0f)
	eigenvalues(0) = 0.0f;
      stat.setZero();
      stat.setEigenVectors(eigenSolver.eigenvectors());
      stat.setEigenValues(eigenSolver.eigenvalues());
      stat.setMean(acc.mean());
      stat.setN(acc.n());
    }
  }
}

void StatsCalculator::compute(NormalVector& normals,
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
      eigenSolver.computeDirect(acc.covariance().block<3,3>(0,0), Eigen::ComputeEigenvectors);

      PointStats &stat = stats[*index];
      Normal& normal = normals[*index];
      const Point&  point = points[*index];
      stat.setZero();
      stat.setEigenVectors(eigenSolver.eigenvectors());
      stat.setMean(acc.mean());
      Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
      if(eigenValues(0) < 0.0f)
	eigenValues(0) = 0.0f;
      stat.setEigenValues(eigenValues);
      stat.setN(acc.n());
      
      normal = stat.block<4, 1>(0, 0);
      if(stat.curvature() < curvatureThreshold) {
	if(normal.dot(point) > 0)
	  normal = -normal;
      } else 
	normal.setZero();
    }
  }
}

}
