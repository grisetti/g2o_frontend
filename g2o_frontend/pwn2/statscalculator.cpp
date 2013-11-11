#include "statscalculator.h"

#include <iostream>
#include <fstream>
#include <set>
#include <omp.h>
#include <Eigen/Eigenvalues>

namespace pwn {

  using namespace boss;

  struct CoordComp {
    bool operator() (const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) const
    {
      return lhs.x() < rhs.x();
    }
  };

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
				const IntImage &intervalImage,
				const IntImage &indexImage) {
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

	assert(*index<(int)statsVector.size());
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
				const IntImage& intervalImage,
				const IntImage& indexImage) {
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
      
	assert(*index<(int)statsVector.size());
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

  void StatsCalculator::fastCompute(NormalVector& normals,
				    InformationMatrixVector &pointInformationMatrix,
				    InformationMatrixVector &normalInformationMatrix,
				    const PointVector& points,
				    const IntImage& indexImage,
				    const int imageRadius) {
    // Generate annulus image coordinates
    std::set<Eigen::Vector2i, CoordComp> annulusCoordinates;
    std::set<Eigen::Vector2i>::iterator it;
    for(int i = 0; i < 90; i++) {
      float rad = M_PI_2 * i / 90.0f;
      Eigen::Vector2i coord;
      coord.x() = imageRadius * sinf(rad); // row 
      coord.y() = imageRadius * cosf(rad); // col
      it = annulusCoordinates.end();
      annulusCoordinates.insert(it, coord);
    }
    
#pragma omp parallel for
    for(int r = 0; r < indexImage.rows(); r++) {
#pragma omp parallel for
      for(int c = 0; c < indexImage.cols(); c++) {
	if(indexImage(r, c) < 0)
	  continue;
	int count = 0;
	Point p = Point(Eigen::Vector3f(0.0, 0.0, 0.0));	
	Normal n = Normal(Eigen::Vector3f(0.0, 0.0, 0.0));
	Eigen::Matrix3f squareP = Eigen::Matrix3f::Zero();
	Eigen::Matrix3f squareN = Eigen::Matrix3f::Zero();
	Eigen::Vector2i centerCoord = Eigen::Vector2i(r, c);
	Point centerPoint = points[indexImage(centerCoord.x(), centerCoord.y())];

#pragma omp parallel for
	for(size_t i = 0; i < annulusCoordinates.size(); i++) {
	  std::set<Eigen::Vector2i>::iterator iterator = annulusCoordinates.begin();
	  std::advance(iterator, i);
	  Eigen::Vector2i currentCoord = *iterator;
	  Eigen::Vector2i firstTranslatedCoord, secondTranslatedCoord;	  
	  
	  // First quadrant	  
	  firstTranslatedCoord = Eigen::Vector2i(-currentCoord.x(), currentCoord.y()) + centerCoord;
	  secondTranslatedCoord = Eigen::Vector2i(-currentCoord.y(), -currentCoord.x()) + centerCoord;
	  if(indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y()) >= 0 &&
	     indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y()) >= 0) {
	    Point firstPoint = points[indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y())];
	    Point secondPoint = points[indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y())];
	    Eigen::Vector3f firstVector = (firstPoint - centerPoint).head<3>();
	    Eigen::Vector3f secondVector = (secondPoint - centerPoint).head<3>();
	    Normal currentNormal = Normal(firstVector.cross(secondVector));
	    currentNormal = currentNormal.normalized();
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    p += firstPoint;
	    n += currentNormal;
	    squareP += firstPoint.head<3>() * firstPoint.head<3>().transpose(); 
	    squareN += currentNormal.head<3>() * currentNormal.head<3>().transpose();
	    count++;
	  }

	  // Second quadrant
	  firstTranslatedCoord = Eigen::Vector2i(-currentCoord.x(), -currentCoord.y()) + centerCoord;
	  secondTranslatedCoord = Eigen::Vector2i(-currentCoord.y(), currentCoord.x()) + centerCoord;	  
	  if(indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y()) >= 0 &&
	     indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y()) >= 0) {
	    Point firstPoint = points[indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y())];
	    Point secondPoint = points[indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y())];
	    Eigen::Vector3f firstVector = (firstPoint - centerPoint).head<3>();
	    Eigen::Vector3f secondVector = (secondPoint - centerPoint).head<3>();
	    Normal currentNormal = Normal(firstVector.cross(secondVector));
	    currentNormal = currentNormal.normalized();
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    p += firstPoint;
	    n += currentNormal;
	    squareP += firstPoint.head<3>() * firstPoint.head<3>().transpose(); 
	    squareN += currentNormal.head<3>() * currentNormal.head<3>().transpose();
	    count++;
	  }

	  // Third quadrant
	  firstTranslatedCoord = Eigen::Vector2i(currentCoord.x(), -currentCoord.y()) + centerCoord;
	  secondTranslatedCoord = Eigen::Vector2i(currentCoord.y(), currentCoord.x()) + centerCoord;	  
	  if(indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y()) >= 0 &&
	     indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y()) >= 0) {
	    Point firstPoint = points[indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y())];
	    Point secondPoint = points[indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y())];
	    Eigen::Vector3f firstVector = (firstPoint - centerPoint).head<3>();
	    Eigen::Vector3f secondVector = (secondPoint - centerPoint).head<3>();
	    Normal currentNormal = Normal(firstVector.cross(secondVector));
	    currentNormal = currentNormal.normalized();
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    p += firstPoint;
	    n += currentNormal;
	    squareP += firstPoint.head<3>() * firstPoint.head<3>().transpose(); 
	    squareN += currentNormal.head<3>() * currentNormal.head<3>().transpose();
	    count++;
	  }

	  // Fourth quadrant
	  firstTranslatedCoord = Eigen::Vector2i(currentCoord.x(), currentCoord.y()) + centerCoord;
	  secondTranslatedCoord = Eigen::Vector2i(currentCoord.y(), -currentCoord.x()) + centerCoord;	  
	  if(indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y()) >= 0 &&
	     indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y()) >= 0) {
	    Point firstPoint = points[indexImage(firstTranslatedCoord.x(), firstTranslatedCoord.y())];
	    Point secondPoint = points[indexImage(secondTranslatedCoord.x(), secondTranslatedCoord.y())];
	    Eigen::Vector3f firstVector = (firstPoint - centerPoint).head<3>();
	    Eigen::Vector3f secondVector = (secondPoint - centerPoint).head<3>();
	    Normal currentNormal = Normal(firstVector.cross(secondVector));
	    currentNormal = currentNormal.normalized();
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    p += firstPoint;
	    n += currentNormal;
	    squareP += firstPoint.head<3>() * firstPoint.head<3>().transpose(); 
	    squareN += currentNormal.head<3>() * currentNormal.head<3>().transpose();
	    count++;
	  }	  
	}
	
	InformationMatrix pInformationMatrix = InformationMatrix::Zero();
	InformationMatrix nInformationMatrix = InformationMatrix::Zero();
	Normal meanN = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	if(count >= 10) {
	  Point meanP = p / (float)count;
	  meanN = n / (float)count;
	  meanN = meanN.normalized();
	  if(n.dot(centerPoint) > 0)
	    meanN = -meanN;
	  pInformationMatrix.block<3, 3>(0, 0) = ((squareP / count) - (meanP.head<3>() * meanP.head<3>().transpose())).inverse();	  
	  nInformationMatrix.block<3, 3>(0, 0) = ((squareN / count) - (meanN.head<3>() * meanN.head<3>().transpose())).inverse();
        }
	
	normals[indexImage(r, c)] = meanN;
	pointInformationMatrix[indexImage(r, c)] = pInformationMatrix;
	normalInformationMatrix[indexImage(r, c)] = nInformationMatrix;
	
	// std::cerr << "Normal: " << normals[indexImage(r, c)].transpose() << std::endl;
	// std::cerr << "pInfo: " << std::endl << pointInformationMatrix[indexImage(r, c)] << std::endl;
	// std::cerr << "nInfo: " << std::endl << normalInformationMatrix[indexImage(r, c)] << std::endl;
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
