#include "statscalculatorcrossproduct.h"

#include <set>

using namespace boss;

namespace pwn {

  StatsCalculatorCrossProduct::StatsCalculatorCrossProduct(int id, 
							   boss::IdContext *context) : StatsCalculator(id, context) {
    _imageRadius = 3;
    _minPoints = 4;
    _normalsTraceThreshold = 1e3;
  }

  struct CoordComp {
    bool operator() (const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) const
    {
      return lhs.x() < rhs.x();
    }
  };

  void StatsCalculatorCrossProduct::compute(NormalVector &normals,
					    StatsVector &statsVector,
					    const PointVector &points,
					    const IntImage &indexImage) {
    assert(indexImage.rows() > 0 && "StatsCalculatorCrossProduct: indexImage has zero rows");
    assert(indexImage.cols() > 0 && "StatsCalculatorCrossProduct: indexImage has zero columns");
    
    if(statsVector.size() != points.size())
      statsVector.resize(points.size());
    if(normals.size() != points.size())
      normals.resize(points.size());
    Normal dummyNormal = Normal::Zero();
    std::fill(normals.begin(), normals.end(), dummyNormal);
    //std::fill(statsVector.begin(), statsVector.end(), Stats());
    
    // Generate annulus image coordinates
    std::set<Eigen::Vector2i, CoordComp> annulusCoordinates;
    std::set<Eigen::Vector2i>::iterator it;
    for(int i = 0; i < _annulusSectionDegrees; i++) {
      float rad = M_PI_2 * i / (float)_annulusSectionDegrees;
      Eigen::Vector2i coord;
      coord.x() = _imageRadius * sinf(rad); // row 
      coord.y() = _imageRadius * cosf(rad); // col
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
	Normal sumN = Normal(Eigen::Vector3f(0.0, 0.0, 0.0));
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
	    n += currentNormal;	    
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    sumN += currentNormal;
	    p += firstPoint;
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
	    n += currentNormal;	    
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    sumN += currentNormal;
	    p += firstPoint;
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
	    n += currentNormal;	    
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    sumN += currentNormal;
	    p += firstPoint;
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
	    n += currentNormal;	    
	    if(currentNormal.dot(centerPoint) > 0)
	      currentNormal = -currentNormal;
	    sumN += currentNormal;
	    p += firstPoint;
	    squareP += firstPoint.head<3>() * firstPoint.head<3>().transpose(); 
	    squareN += currentNormal.head<3>() * currentNormal.head<3>().transpose();
	    count++;
	  }	  
	}
	
	// InformationMatrix pInformationMatrix = InformationMatrix::Zero();
	// InformationMatrix nInformationMatrix = InformationMatrix::Zero();
	Normal meanNormals = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	Normal meanN = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	Stats stats = Stats();
	stats.setZero();
	// Add stats filler
	if(count >= _minPoints) {
	  // Point meanP = p / (float)count;
	  // meanNormals = n / (float)count;
	  // pInformationMatrix.block<3, 3>(0, 0) = ((squareP / (float)count) - (centerPoint.head<3>() * centerPoint.head<3>().transpose())).inverse();	  	  
	  // nInformationMatrix.block<3, 3>(0, 0) = ((squareN / (float)count) - (meanNormals.head<3>() * meanNormals.head<3>().transpose())).inverse();

	  // if(fabs(nInformationMatrix(0, 0)*nInformationMatrix(1, 1)*nInformationMatrix(2, 2)) > 1e03) {
	  //   nInformationMatrix = Eigen::Matrix4f::Zero();
	  //   nInformationMatrix(0, 0) = 100.0f;
	  //   nInformationMatrix(1, 1) = 100.0f;
	  //   nInformationMatrix(2, 2) = 100.0f;
	  // }
	  // else {
	  //   nInformationMatrix = Eigen::Matrix4f::Identity();
	  // }
	  meanN = (sumN / (float)count).normalized();	  	  
	  if(meanN.dot(centerPoint) > 0)
	    meanN = -meanN;
	}
	
	normals[indexImage(r, c)] = meanN;
	statsVector[indexImage(r, c)] = stats;
	// pointInformationMatrix[indexImage(r, c)] = pInformationMatrix;
	// normalInformationMatrix[indexImage(r, c)] = nInformationMatrix;
      }
    }
  }

  void StatsCalculatorCrossProduct::serialize(boss::ObjectData &data, boss::IdContext &context) {
    StatsCalculator::serialize(data, context);
    Identifiable::serialize(data, context);
    data.setInt("imageRadius", imageRadius());
    data.setInt("minPoints", minPoints());
    data.setFloat("normalsTraceThreshold", normalsTraceThreshold());    
  }

  void StatsCalculatorCrossProduct::deserialize(boss::ObjectData &data, boss::IdContext &context){
    StatsCalculator::deserialize(data, context);
    Identifiable::deserialize(data, context);
    setImageRadius(data.getInt("imageRadius"));
    setMinPoints(data.getInt("minPoints"));
    setNormalTraceThreshold(data.getFloat("normalsTraceThreshold"));
  }
  
  BOSS_REGISTER_CLASS(StatsCalculatorCrossProduct);
}
