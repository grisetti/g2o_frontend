#include "statscalculatorintegralimage.h"

#include <Eigen/Eigenvalues> 

using namespace boss;

namespace pwn {
  StatsCalculatorIntegralImage::StatsCalculatorIntegralImage(int id, 
							     IdContext *context) : StatsCalculator(id, context) {
    _worldRadius = 0.1f;
    _maxImageRadius = 30;
    _minImageRadius = 10;
    _minPoints = 50;
    _curvatureThreshold = 0.2f;
  }

  void StatsCalculatorIntegralImage::compute(NormalVector& normals,
					     StatsVector& statsVector,
					     const PointVector& points,
					     const IntImage& indexImage) {
    assert(indexImage.rows() > 0 && "StatsCalculatorIntegralImage: indexImage has zero rows");
    assert(indexImage.cols() > 0 && "StatsCalculatorIntegralImage: indexImage has zero columns");
    assert(_intervalImage.rows() > 0 && "StatsCalculatorIntegralImage: _intervalImage has zero rows");
    assert(_intervalImage.cols() > 0 && "StatsCalculatorIntegralImage: _intervalImage has zero columns");

    if(indexImage.rows() != _intervalImage.rows() || indexImage.cols() == _intervalImage.cols()) {
      _integralImage.resize(indexImage.rows(), indexImage.cols());
    }

    if(statsVector.size() != points.size())
      statsVector.resize(points.size());
    if(normals.size() != points.size())
      normals.resize(points.size());
    Normal dummyNormal = Normal::Zero();
    std::fill(normals.begin(), normals.end(), dummyNormal);
    //std::fill(statsVector.begin(), statsVector.end(), Stats());
    
    // Computing the integral image
    _integralImage.compute(indexImage, points);    

#pragma omp parallel for
    for(int c = 0; c < indexImage.cols(); ++c) {
      const int *index = &indexImage.coeffRef(0, c);
      const int *interval = &_intervalImage.coeffRef(0, c);
      for(int r = 0; r < indexImage.rows(); ++r, ++index, ++interval) {
     
	// is the point valid, is its range valid?
	if(*index<0 || *interval<0)
	  continue;
      
	assert(*index<(int)statsVector.size());
	int imageRadius = *interval;
	if(imageRadius < _minImageRadius)
	  imageRadius = _minImageRadius;
	if(imageRadius > _maxImageRadius)
	  imageRadius = _maxImageRadius;

	const PointAccumulator acc = _integralImage.getRegion(r - imageRadius, r + imageRadius,
							      c - imageRadius, c + imageRadius);
	if(acc.n() < _minPoints)
	  continue;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
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
      }
    }
  }

  void StatsCalculatorIntegralImage::serialize(boss::ObjectData &data, boss::IdContext &context) {
    StatsCalculator::serialize(data, context);
    Identifiable::serialize(data, context);
    data.setFloat("worldRadius", worldRadius());
    data.setInt("imageMaxRadius", maxImageRadius());
    data.setInt("imageMinRadius", minImageRadius());
    data.setInt("minPoints", minPoints());
    data.setFloat("curvatureThreshold", curvatureThreshold());
  }

  void StatsCalculatorIntegralImage::deserialize(boss::ObjectData &data, boss::IdContext &context){
    StatsCalculator::deserialize(data, context);
    Identifiable::deserialize(data, context);
    setWorldRadius(data.getFloat("worldRadius"));
    setMaxImageRadius(data.getInt("imageMaxRadius"));
    setMinImageRadius(data.getInt("imageMinRadius"));
    setMinPoints(data.getInt("minPoints"));
    setCurvatureThreshold(data.getFloat("curvatureThreshold"));
  }

  BOSS_REGISTER_CLASS(StatsCalculatorIntegralImage);
}
