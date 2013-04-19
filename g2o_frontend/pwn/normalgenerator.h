#ifndef _HOMOGENEOUSPOINT3FNORMALGENERATOR_H_
#define _HOMOGENEOUSPOINT3FNORMALGENERATOR_H_

#include "depthimage.h"
#include "homogeneousvector4f.h"
#include "pinholepointprojector.h"
#include "homogeneouspoint3fstatsgenerator.h"
#include <iostream>

using namespace Eigen;

struct NormalGenerator {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Empty constructor.
  // Set the rows and the cols of the depth image to use to zero.
  NormalGenerator() {
    scaledRows = 0;
    scaledCols = 0;
  }

  // Method that computes the points with normals given a the depth image.
  // The resulting points and normals are stored in the first two input
  // parameters, the third parameter is the input depth image.
  void compute(HomogeneousPoint3fVector& imagePoints, HomogeneousNormal3fVector& imageNormals, const DepthImage& depthImage, float curvatureThreshold, Isometry3f offset = Isometry3f::Identity()) {
    // Update the size of the index image.
    indexImage.resize(depthImage.rows(), depthImage.cols());
    
    // Before to update the scaled rows and columns save the previous values.
    int oldScaledRows = scaledRows; 
    int oldScaledCols = scaledCols;
    scaledRows = depthImage.rows() * scale;
    scaledCols = depthImage.cols() * scale;

    // If the scaling factor changed then recompute the scaled images.
    if(oldScaledCols != scaledCols || oldScaledRows != scaledRows) {
      scaledDepthImage.resize(scaledRows, scaledCols);
      scaledIndexImage.resize(scaledRows, scaledCols);
      scaledIntervalImage.resize(scaledRows, scaledCols);
    }
    
    projector.setTransform(offset);
    
    // Set the camera matrix of the projector object.
    projector.setCameraMatrix(cameraMatrix);

    // Get the points in the 3d euclidean space.
    projector.unProject(imagePoints, indexImage, depthImage);

    // Step one, set the scaled camera matrix for the projector object.
    projector.setCameraMatrix(scaledCameraMatrix);
    // Step two, rescale the image to based on the setted scale factor 
    // and obtain a projection on the image plane.
    projector.project(scaledIndexImage, scaledDepthImage, imagePoints);
    
    // Step three, compute the integral image.
    scaledIntegralImage.compute(scaledIndexImage, imagePoints);

    // Step four, compute the intervals.
    projector.projectIntervals(scaledIntervalImage, scaledDepthImage, 0.1f);
    
    // Resize the vector containing the stats to have the same length of the vector of points.
    scaledStats.resize(imagePoints.size());
    std::fill(scaledStats.begin(), scaledStats.end(), HomogeneousPoint3fStats());
    
    // Step five, compute the statistics and extract the normals.
    statsGenerator.compute(scaledStats, scaledIntegralImage, scaledIntervalImage, scaledIndexImage);

    // Step six, extract the normals from the statistics.
    imageNormals.resize(imagePoints.size());
    for(size_t i = 0; i < imagePoints.size(); i++) {
      HomogeneousPoint3f& p = imagePoints[i];
      const HomogeneousPoint3fStats& pstats = scaledStats[i];
      HomogeneousNormal3f& normal = imageNormals[i];
      normal = scaledStats[i].block<4, 1>(0, 0);
      if(pstats.curvature() < curvatureThreshold) {
	if(normal.dot(p) > 0)
	  normal = -normal;
      } else {
	normal.setZero();
      }
    }
  }

  // This method allows to set the camera matrix.
  void setCameraMatrix(const Matrix3f cameraMatrix_) {
    cameraMatrix = cameraMatrix_;
    updateCamera();
  }
  
  // This method allows to set the depth image scale factor.
  void setScale(float scale_) {
    scale = scale_;
    updateCamera();
  }

  // This method update the scaled camera matrix used for the
  // scaled depth image.
  void updateCamera() {
    scaledCameraMatrix = cameraMatrix;
    scaledCameraMatrix.block<2,3>(0, 0) *= scale;
  }
  
  // Variable hosting the scaled depth image.
  DepthImage scaledDepthImage;
  // Variables hosting the indices of the scaled and normal size depth images.
  MatrixXi indexImage, scaledIndexImage;
  // Variable hosting the radius of the ball in pixels at each point where to compute the normals.
  MatrixXi scaledIntervalImage;
  // Variable hosting the integral image used to fastly compute the point statistics on the 
  // scaled depth image.
  HomogeneousPoint3fIntegralImage scaledIntegralImage;
  // Variable hostinghosts the statistics of the points neighborhood for the scaled depth image: 
  // eigenvalues, eigenvectors and curvature.
  HomogeneousPoint3fStatsVector scaledStats;
  
  // This object projects the points (or their indices) onto an image (and viceversa).
  PinholePointProjector projector;

  // This object computes the statistics from an integral image.
  HomogeneousPoint3fStatsGenerator statsGenerator;

  // Camera matrix used for points projection/unprojection.
  Matrix3f cameraMatrix;

  // Variables used for the scaling. 
  float scale;
  int scaledRows;
  int scaledCols;
  Matrix3f scaledCameraMatrix;
};

#endif
