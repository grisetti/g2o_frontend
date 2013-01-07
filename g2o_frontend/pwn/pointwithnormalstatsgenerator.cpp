#include <iostream>
#include "pointwithnormalstatsgenerator.h"
#include "Eigen/SVD"
using namespace std;

PointWithNormalStatistcsGenerator::PointWithNormalStatistcsGenerator(){
  _step = 1;
  _worldRadius = 0.1;
  _imageRadius = 100;
  _minPoints = 50;
  _maxCurvature = 0.5f;
  _cameraMatrix <<   
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
}


  // void PointWithNormalStatistcsGenerator::computeNormals(PointWithNormalVector& points, const Eigen::MatrixXi indices, const Eigen::Matrix3f& cameraMatrix);

void PointWithNormalStatistcsGenerator::computeNormalsAndSVD(PointWithNormalVector& points, PointWithNormalSVDVector& svds, const Eigen::MatrixXi& indices, const Eigen::Matrix3f& cameraMatrix){
  _integralImage.compute(indices,points);
  int q=0;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver;
  for (int c=0; c<indices.cols(); c+=_step)
    for (int r=0; r<indices.rows(); r+=_step, q++){
      int index  = indices(r,c);
      //cerr << "index("  << r <<"," << c << ")=" << index <<  endl;
      if (index<0)
	continue;
      // determine the region
      PointWithNormal& point = points[index];
      PointWithNormalSVD& svd = svds[index];
      Eigen::Vector3f coord = cameraMatrix * (point.point()+Eigen::Vector3f(_worldRadius, _worldRadius, 0));
      coord.head<2>()*=(1./coord(2));
      int dx = abs(c - coord[0]);
      int dy = abs(r - coord[1]);
      if (dx>_imageRadius)
	dx = _imageRadius;
      if (dy>_imageRadius)
	dy = _imageRadius;
      PointAccumulator acc = _integralImage.getRegion(c-dx, c+dx, r-dy, r+dy);
      if (acc.n()>_minPoints){
	Eigen::Vector3f mean = acc.mean();
	Eigen::Matrix3f cov  = acc.covariance();
	eigenSolver.compute(cov);
	svd._U=eigenSolver.eigenvectors();
	svd._singularValues=eigenSolver.eigenvalues();
	if (svd._singularValues(0) <0)
	  svd._singularValues(0) = 0;
	/*
	cerr << "\t svd.singularValues():" << svd.singularValues() << endl;
	cerr << "\t svd.U():" << endl << svd.U() << endl;
	//cerr << "\t svd.curvature():" << svd.curvature() << endl;
	
	*/
	Eigen::Vector3f normal = eigenSolver.eigenvectors().col(0).normalized();
	if (normal.dot(mean) > 0.0f)
	  normal =-normal;
	point.setNormal(normal);
	//cerr << "n(" << index << ") c:"  << svd.curvature() << endl << point.tail<3>() << endl;
	if (svd.curvature()>_maxCurvature){
	  //cerr << "region: " << c-dx << " " <<  c+dx << " " <<  r-dx << " " << r+dx << " points: " << acc.n() << endl;
	  point.tail<3>().setZero();
	} 
      } else {
	point.tail<3>().setZero();
	svd = PointWithNormalSVD();
      }
    } 
}
  
