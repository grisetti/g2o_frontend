#include "ransac.h"
#include "g2o/types/slam2d/types_slam2d.h"

#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <vector>
#include <map>
#include <iostream>

using namespace g2o_frontend;
using namespace g2o;
using namespace Eigen;
using namespace std;

  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dVector;
  
  template <int D>
  struct SVDDescriptor{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Matrix<double, D, 1> VectorType;
    typedef Eigen::Matrix<double, D, D> MatrixType;
    VectorType singularValues;
    MatrixType UV;
    int nPoints;
    double distance (const SVDDescriptor& other) const {
      VectorType dsv = singularValues - other.singularValues;
      return dsv.norm()/sqrt(nPoints);
    }
  };  

template <int D>
ostream & operator << (ostream& os, const SVDDescriptor<D>&d) {
  os << d.singularValues(0) << " " << d.singularValues(1) << " " << d.nPoints;
  return os;
}

  typedef SVDDescriptor<2> SVDDescriptor2;

  typedef std::vector<SVDDescriptor2, Eigen::aligned_allocator<SVDDescriptor2> > SVDDescriptor2Vector;

void computeSVDDescriptors2(SVDDescriptor2Vector& descriptors, const Vector2dVector& points, double squaredRadius, int maxPoints) {
    JacobiSVD<Matrix2d> svd;
    descriptors.resize(points.size());
    Vector2dVector neighbors;
    neighbors.reserve(points.size());
    for (size_t i=0; i<points.size(); i++){
      const Vector2d& pi = points[i];
      neighbors.clear();
      std::multimap<double, int> distanceMap;
      // mean and neighbors;
      for (size_t j=0; j<points.size(); j++){
	const Vector2d& pj = points[j];
	double d = (pj-pi).squaredNorm();
	if ( d<squaredRadius){
	  distanceMap.insert(make_pair(d,j));
	}
      }
      int k=0;
      Vector2d mean(0.,0.);
      for (std::multimap<double, int>::iterator it = distanceMap.begin();k <maxPoints && it!=distanceMap.end(); it++) {
	const Vector2d& pj=points[it->second];
	neighbors.push_back(pj);
	mean += pj;
	k++;
      }
      cerr << "INDEX: " << i 
	   <<  " distanceMap.size()" << distanceMap.size() 
	   << " neighbors.size():" << neighbors.size() << endl;
      double icount = 1./(double)neighbors.size();
      mean *=icount;
      Matrix2d sigma=Matrix2d::Zero(); 
      for (size_t j=0; j<neighbors.size(); j++){
	const Vector2d& pj=neighbors[j];
	Vector2d dp = pj-mean;
	sigma += dp*dp.transpose();
      }
      sigma *= icount;
      svd.compute(sigma, Eigen::ComputeThinU | Eigen::ComputeThinV);
      descriptors[i].singularValues = svd.singularValues();
      descriptors[i].UV = svd.matrixU()*svd.matrixV().transpose();
      descriptors[i].nPoints = neighbors.size();
    }
  }


  int main(){
    size_t nPoints = 100;
    double noise=.5;
    double radius = 9;
    double threshold = 0.001;
    int maxPoints = 5;
    SVDDescriptor2Vector descriptors1(nPoints), descriptors2(nPoints);
    Vector2dVector points1(nPoints), points2(nPoints);
    
    double scale = 10;
    points1.resize(nPoints);
    
    
    SE2 T(100, 50, .5);
    for (size_t i=0; i<nPoints; i++){
      Vector2d v;
      v.x()= scale * (drand48()-.5);
      v.y()= scale * (drand48()-.5);
      points1[i] = v;
      Vector2d n=v;
      n.x()+= noise * (drand48()-.5);
      n.y()+= noise * (drand48()-.5);
      n = T*n;
      points2[i] = n;
    }
 

    cerr << "computing descriptors" << endl;
    computeSVDDescriptors2(descriptors1, points1, radius, maxPoints);
    computeSVDDescriptors2(descriptors2, points2, radius, maxPoints);

    int outliers = 0;
    int inliers = 0;
    int dualMatchInliers = 0;
    cerr << "computing matches" << endl;
    for (size_t i =0; i< descriptors1.size() ; i++){
      const SVDDescriptor2& d1=descriptors1[i];
      if (d1.nPoints<2)
	continue;
      int bestIndex = -1;
      double bestDistance= std::numeric_limits<double>::max();
      
      int secondBestIndex = -1;
      double secondBestDistance= std::numeric_limits<double>::max();
      
      for (size_t j =0; j< descriptors2.size(); j++){
	const SVDDescriptor2& d2=descriptors1[j];
	if (d2.nPoints<2)
	  continue;
      	double currentDistance = d1.distance(d2);
	if (bestIndex<0 || currentDistance<bestDistance){
	  secondBestIndex = bestIndex;
	  secondBestDistance = bestDistance;
	  bestIndex = j;
	  bestDistance = currentDistance;
	}
      }
      
      if(secondBestDistance - bestDistance > threshold){
	cerr << "index" << i << " best match: " << bestIndex <<  " UNIQUE distance:"  << bestDistance << endl;
	cerr << "\t d1:" << descriptors1[i]  << endl;
	cerr << "\t d2:" << descriptors2[bestIndex]  << endl;

	if (bestIndex == i){
	  inliers ++;
	} else {
	  outliers++;
	}
      } else {
	//cerr << "index" << i << " best match: " << bestIndex <<  " MESSED" <<  endl;
      }
    }
    

    cerr << "true inliers: " << inliers << endl;
    cerr << "true outliers: " << outliers << endl;
    cerr << "true unclassified: " << nPoints - inliers - outliers << endl;

    
  }

