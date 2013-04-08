#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include "homogeneousvector4f.h"
#include "../basemath/bm_se3.h"
#include "depthimage.h"
#include "pinholepointprojector.h"
#include "homogeneouspoint3fintegralimage.h"
#include "homogeneouspoint3fstatsgenerator.h"
#include "Eigen/SVD"
#include "pointwithnormal.h"
#include "g2o/stuff/timeutil.h"

using namespace Eigen;
using namespace std;

struct DoNormalsForIgorAndOlga {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DoNormalsForIgorAndOlga(){
    scaledRows = 0;
    scaledCols = 0;
  }
  void compute(  
	       HomogeneousPoint3fVector& imagePoints,
	       HomogeneousNormal3fVector& imageNormals,
	       const DepthImage& depthImage){

    indexImage.resize(depthImage.rows(), depthImage.cols());
    int oldScaledRows = scaledRows, oldScaledCols=scaledCols;
    
    scaledRows = depthImage.rows()* scale;
    scaledCols = depthImage.cols()* scale;

    if (oldScaledCols!=scaledCols || oldScaledRows != scaledRows) {
      // hosts the indices of the scaled image
      scaledDepthImage.resize(scaledRows, scaledCols);
      // hosts the indices of the scaled image
      scaledIndexImage.resize(scaledRows, scaledCols);
      // hosts the radius of the ball in pixels at each point where to compute the normals
      scaledIntervalImage.resize(scaledRows, scaledCols);
      // hosts the integral image used to fastly compute the point statistics
      HomogeneousPoint3fIntegralImage scaledIntegralImage;
    }
    
    projector.setCameraMatrix(cameraMatrix);
    projector.unProject(imagePoints,indexImage,depthImage);

    // step one, get the points from the raw depth image
    // cerr <<"rescale" << endl;
    // step two rescale the image to 1/2 of the size and obtain a projection
    projector.setCameraMatrix(scaledCameraMatrix);
    projector.project(scaledIndexImage,scaledDepthImage,imagePoints);
    
    // cerr <<"integral" << endl;
    // step three compute the integral image
    scaledIntegralImage.compute(scaledIndexImage,imagePoints);

    // cerr << "intervals" << endl;
    // step four compute the intervals
    projector.projectIntervals(scaledIntervalImage, scaledDepthImage, 0.1f);
    // cerr << "scaledIntervalImage.size() = " << scaledIntervalImage.rows() <<"x" << scaledIntervalImage.cols() << endl;
    // cerr << "scaledIndexImage.size() = " << scaledIndexImage.rows() <<"x" << scaledIndexImage.cols() << endl;
    // cerr << "scaledIntegralImage.size() = " << scaledIntegralImage.rows() <<"x" << scaledIntegralImage.cols() << endl;
    // cerr << "scaledStats.size() = " << scaledStats.size() << endl;
    // cerr << "stats" << endl;
    scaledStats.resize(imagePoints.size());
    // step five, compute the statistics and extract the normals
    statsGenerator.compute(scaledStats, scaledIntegralImage, scaledIntervalImage, scaledIndexImage);

    // step six, extract the normals from the statistics
    imageNormals.resize(imagePoints.size());
    
    // cerr << "assign" << endl;
    for(size_t i=0; i<imagePoints.size(); i++){
      const HomogeneousPoint3f& p = imagePoints[i];
      const HomogeneousPoint3fStats& pstats = scaledStats[i];
      HomogeneousNormal3f& normal = imageNormals[i];
      normal=scaledStats[i].block<4,1>(0,0);
      if (pstats.curvature() < 0.02) {
	if (normal.dot(p)>0)
	  normal = -normal;
      } else {
	normal.setZero();
      }
    }

  }

  void setCameraMatrix(const Eigen::Matrix3f cameraMatrix_){
    cameraMatrix=cameraMatrix_;
    updateCamera();
  }
  
  void setScale(float scale_) {
    scale=scale_;
    updateCamera();
  }

  void updateCamera(){
    scaledCameraMatrix = cameraMatrix;
    scaledCameraMatrix.block<2,3>(0,0)*=scale;
  }
  
  // hosts the indices of the scaled image
  DepthImage scaledDepthImage;
  // hosts the indices of the scaled image
  Eigen::MatrixXi indexImage, scaledIndexImage;
  // hosts the radius of the ball in pixels at each point where to compute the normals
  Eigen::MatrixXi scaledIntervalImage;
    // hosts the integral image used to fastly compute the point statistics
  HomogeneousPoint3fIntegralImage scaledIntegralImage;
  // hosts the statistics of the point neighborhood, eigenvalues eigenvectors and curvature
  HomogeneousPoint3fStatsVector scaledStats;
  
  // this object projects the points (or their indices) onto an image (and viceversa)
  PinholePointProjector projector;

  // this object computes the statistrics for an integral image
  HomogeneousPoint3fStatsGenerator statsGenerator;

  // camera matrix for the kinect
  Eigen::Matrix3f cameraMatrix;

  // variables for the scaling 
  float scale;
  int scaledRows;
  int scaledCols;
  Eigen::Matrix3f scaledCameraMatrix;
};

int  main(int argc , char** argv){

  size_t numTrials =100;
  DepthImage image1;

  // this loads the depth image
  if (argc>1){
    if (!image1.load(argv[1])){
      cerr << "error loading image [" << argv[1] << "]" << endl;
      return 0;
    }
  }
  // this is an hack since in the old code the images are loade column-wise. 
  image1.transposeInPlace();
  cerr << "loaded image size: " << image1.rows() << "x" << image1.cols() << endl;


  DoNormalsForIgorAndOlga normalThing;
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix <<     
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  normalThing.setScale(0.5f);
  normalThing.setCameraMatrix(cameraMatrix);

  // here your points will go
  HomogeneousPoint3fVector imagePoints; 
  // here your normals will go
  HomogeneousNormal3fVector imageNormals;

  double t0=g2o::get_time();
  for (size_t i=0; i<numTrials; i++){
    normalThing.compute(imagePoints, imageNormals, image1);
    cerr << ".";
  }
  double t1=g2o::get_time();


  cerr << endl << "total time for normal computation:" << double(t1-t0)/double(numTrials) << endl;
  
  // this is just to check that the result is correct
  PointWithNormalVector pnv(imagePoints.size());
  for (size_t i=0; i<pnv.size(); ++i){
    pnv[i].head<3>()=imagePoints[i].head<3>();
    pnv[i].tail<3>()=imageNormals[i].head<3>();
  }
  pnv.save("out.pwn",true);
}

