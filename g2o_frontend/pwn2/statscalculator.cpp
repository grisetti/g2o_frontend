#include "statscalculator.h"
#include <Eigen/Eigenvalues>
#include <omp.h>

#include <iostream>
#include <fstream>

namespace pwn {
  using namespace boss;

  StatsCalculator::StatsCalculator(int id, IdContext* context): Identifiable(id,context){
    _worldRadius = 0.1;
    _maxImageRadius = 30;
    _minImageRadius = 10;
    _minPoints = 50;
    _curvatureThreshold = 0.2;
  }

  StatsCalculator::~StatsCalculator(){}

  void StatsCalculator::compute(StatsVector &statsVector,
				const PointIntegralImage &integralImage,
				const Eigen::MatrixXi &intervalImage,
				const Eigen::MatrixXi &indexImage) {
    assert(integralImage.rows() == intervalImage.rows());
    assert(integralImage.cols() == intervalImage.cols());
    assert(integralImage.rows() == indexImage.rows());
    assert(integralImage.cols() == indexImage.cols());

#pragma omp parallel for
    for(int c = 0; c < indexImage.cols(); ++c) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
      const int *index = &indexImage.coeffRef(0,c);
      const int *interval = &intervalImage.coeffRef(0,c);
      for(int r = 0; r < indexImage.rows(); ++r, ++index, ++interval) {
     
	// is the point valid, is its range valid?
	if(*index < 0 || *interval < 0)
	  continue;

	assert(*index<(int)stats.size());
	int imageRadius = *interval;
	if(imageRadius < _minImageRadius)
	  imageRadius = _minImageRadius;
	if(imageRadius > _maxImageRadius)
	  imageRadius = _maxImageRadius;

	PointAccumulator acc = integralImage.getRegion(r - imageRadius, r + imageRadius,
						       c - imageRadius, c + imageRadius);
	if(acc.n() < _minPoints)
	  continue;
	Stats& stats = statsVector[*index];
	Eigen::Matrix3f covariance3f = acc.covariance().block<3,3>(0,0);
	eigenSolver.computeDirect(covariance3f, Eigen::ComputeEigenvectors);
	Eigen::Vector4f eigenvalues;
	if (eigenvalues(0) < 0.0f)
	  eigenvalues(0) = 0.0f;
	stats.setZero();
	stats.setEigenVectors(eigenSolver.eigenvectors());
	stats.setEigenValues(eigenSolver.eigenvalues());
	stats.setMean(acc.mean());
	stats.setN(acc.n());
      }
    }
  }

  void StatsCalculator::compute(NormalVector& normals,
				StatsVector& statsVector,
				const PointVector& points,
				const PointIntegralImage& integralImage,
				const Eigen::MatrixXi& intervalImage,
				const Eigen::MatrixXi& indexImage) {
    assert(integralImage.rows() == intervalImage.rows());
    assert(integralImage.cols() == intervalImage.cols());
    assert(integralImage.rows() == indexImage.rows());
    assert(integralImage.cols() == indexImage.cols());

    if(statsVector.size() != points.size())
      statsVector.resize(points.size());
    if(normals.size() != points.size())
      normals.resize(points.size());
  
    Normal zero;
    zero.setZero();  
    std::fill(statsVector.begin(), statsVector.end(), Stats());
    std::fill(normals.begin(), normals.end(), zero);

#pragma omp parallel for
    for(int c = 0; c < indexImage.cols(); ++c) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
      const int *index = &indexImage.coeffRef(0, c);
      const int *interval = &intervalImage.coeffRef(0, c);
      for(int r = 0; r < indexImage.rows(); ++r, ++index, ++interval) {
     
	// is the point valid, is its range valid?
	if(*index<0 || *interval<0)
	  continue;
      
	assert(*index<(int)stats.size());
	int imageRadius = *interval;
	if(imageRadius < _minImageRadius)
	  imageRadius = _minImageRadius;
	if(imageRadius > _maxImageRadius)
	  imageRadius = _maxImageRadius;

	const PointAccumulator acc = integralImage.getRegion(r - imageRadius, r + imageRadius,
							     c - imageRadius, c + imageRadius);
	if(acc.n() < _minPoints)
	  continue;
	eigenSolver.computeDirect(acc.covariance().block<3, 3>(0, 0), Eigen::ComputeEigenvectors);

	Stats &stats = statsVector[*index];
	Normal &normal = normals[*index];
	const Point &point = points[*index];
	stats.setZero();
	stats.setEigenVectors(eigenSolver.eigenvectors());
	stats.setMean(acc.mean());
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
	if(eigenValues(0) < 0.0f)
	  eigenValues(0) = 0.0f;
	stats.setEigenValues(eigenValues);
	stats.setN(acc.n());
      
	normal = stats.block<4, 1>(0, 0);
	if(stats.curvature() < _curvatureThreshold) {
	  if(normal.dot(point) > 0)
	    normal = -normal;
	} else
	  normal.setZero();
      
	if (0){
	  int ira=-1;
	  int ica=-1;
	  int irb=-1;
	  int icb=-1;
	  int d=5;
	  if (r>=d && r < indexImage.rows()-d && c >= d && c < indexImage.cols()-d){
	    ira = indexImage(r+d,c);
	    irb = indexImage(r-d,c);
	    ica = indexImage(r,c+d);
	    icb = indexImage(r,c-d);
	  }
	  normal.setZero();
	  if (ira >0 && irb >0 && ica >0 && icb>0) {
	    const Point _dx=points[ira]-points[irb];
	    const Point _dy=points[ica]-points[icb];
	    Eigen::Vector3f dx=_dx.head<3>();
	    Eigen::Vector3f dy=_dy.head<3>();
	    Eigen::Vector3f n=dy.cross(dx);
	    n.normalize();
	    normal = n;
	  }
	}
	
      
      }
    }
  }

  void StatsCalculator::serialize(boss::ObjectData& data, boss::IdContext& context) {
    Identifiable::serialize(data,context);
    data.setFloat("worldRadius", worldRadius());
    data.setInt("imageMaxRadius", maxImageRadius());
    data.setInt("imageMinRadius", minImageRadius());
    data.setInt("minPoints", minPoints());
    data.setFloat("curvatureThreshold", curvatureThreshold());
  }

  void StatsCalculator::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
    setWorldRadius(data.getFloat("worldRadius"));
    setMaxImageRadius(data.getFloat("imageMaxRadius"));
    setMinImageRadius(data.getFloat("imageMinRadius"));
    setMinPoints(data.getInt("minPoints"));
    setCurvatureThreshold(data.getFloat("curvatureThreshold"));
  }

  
  BOSS_REGISTER_CLASS(StatsCalculator);

}
